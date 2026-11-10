#include "ompl_bridge/src/bridge.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/geometric/PathHybridization.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>

#include "ompl_bridge/src/ffi.rs.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_bridge {
namespace {

using Clock = std::chrono::steady_clock;

// The seed of attempt `k` of a plan seeded with `seed`: the seed itself for attempt 0,
// and for each other a mix of the two by SplitMix64's finalizer, never 0, which OMPL does
// not take as a seed. A plan of several attempts joins their paths in a problem seeded
// with the seed of index `attempts`, one past its last attempt's.
std::uint32_t attempt_seed(std::uint32_t seed, std::uint32_t k) {
  if (k == 0) {
    return seed;
  }
  std::uint64_t z = ((static_cast<std::uint64_t>(seed) << 32) | k) + 0x9E3779B97F4A7C15ULL;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  const auto mixed = static_cast<std::uint32_t>(z ^ (z >> 31));
  return mixed == 0 ? 1 : mixed;
}

// The values of a state of the plan's space, a real vector space or the space
// projected onto the constraint.
const double* values(const ob::State* state, bool constrained) {
  if (constrained) {
    return state->as<ob::ConstrainedStateSpace::StateType>()->data();
  }
  return state->as<ob::RealVectorStateSpace::StateType>()->values;
}

class StateCheck : public ob::StateValidityChecker {
 public:
  StateCheck(const ob::SpaceInformationPtr& si, const Callbacks& callbacks, std::size_t dimension,
             bool constrained)
      : ob::StateValidityChecker(si),
        callbacks_(callbacks),
        dimension_(dimension),
        constrained_(constrained) {}

  bool isValid(const ob::State* state) const override {
    return callbacks_.state_valid(
        rust::Slice<const double>(values(state, constrained_), dimension_));
  }

 private:
  const Callbacks& callbacks_;
  std::size_t dimension_;
  bool constrained_;
};

// The caller's check of a straight motion between two states of the plan's space, the
// planner's or, with `simplifying`, the simplifier's, and the counts OMPL keeps of a
// validator's answers.
class CallerMotion : public ob::MotionValidator {
 public:
  CallerMotion(const ob::SpaceInformationPtr& si, const Callbacks& callbacks,
               std::size_t dimension, bool constrained, bool simplifying)
      : ob::MotionValidator(si),
        dimension_(dimension),
        callbacks_(callbacks),
        constrained_(constrained),
        simplifying_(simplifying) {}

 protected:
  bool check(const ob::State* from, const ob::State* to, double& last_valid) const {
    return callbacks_.motion_valid(
        rust::Slice<const double>(values(from, constrained_), dimension_),
        rust::Slice<const double>(values(to, constrained_), dimension_), simplifying_,
        last_valid);
  }

  bool count(bool valid) const {
    if (valid) {
      ++valid_;
    } else {
      ++invalid_;
    }
    return valid;
  }

  std::size_t dimension_;

 private:
  const Callbacks& callbacks_;
  bool constrained_;
  bool simplifying_;
};

// The caller's check of the straight motion between two states of a real vector
// space.
class MotionCheck : public CallerMotion {
 public:
  MotionCheck(const ob::SpaceInformationPtr& si, const Callbacks& callbacks, std::size_t dimension,
              bool simplifying)
      : CallerMotion(si, callbacks, dimension, false, simplifying) {}

  bool checkMotion(const ob::State* from, const ob::State* to) const override {
    double last_valid = 0.0;
    return count(check(from, to, last_valid));
  }

  bool checkMotion(const ob::State* from, const ob::State* to,
                   std::pair<ob::State*, double>& last_valid) const override {
    double fraction = 0.0;
    const bool valid = check(from, to, fraction);
    if (!valid) {
      if (last_valid.first != nullptr) {
        si_->getStateSpace()->interpolate(from, to, fraction, last_valid.first);
      }
      last_valid.second = fraction;
    }
    return count(valid);
  }
};

// How far a step of a walk along the constraint may stand from the line of the stretch it
// extends and still extend it, the bound motion_planning_core's `on_segment` holds a run of a
// path's answer to: steps the projection leaves where they are round by some ulps of 4.4e-16 rad
// each, under this over a million steps.
constexpr double stretch_tolerance = 1e-9;

// The caller's check of a motion along the constraint: OMPL's walk of the motion, as its
// own `ConstrainedMotionValidator` walks it, with the caller's check of each straight
// stretch of the walk in place of its check of each state. A motion the walk does not
// finish is invalid, and so is one with a state outside the space's bounds, which the walk
// ends at: OMPL's walk leaves the bounds to the validity check it skips here, as MoveIt's
// validity check holds them (state_validity_checker.cpp:233). An invalid motion's last valid
// state is the end of its last stretch that passes.
class WalkedMotionCheck : public CallerMotion {
 public:
  WalkedMotionCheck(const ob::SpaceInformationPtr& si, const Callbacks& callbacks,
                    std::size_t dimension, bool simplifying)
      : CallerMotion(si, callbacks, dimension, true, simplifying) {}

  bool checkMotion(const ob::State* from, const ob::State* to) const override {
    std::pair<ob::State*, double> last_valid{nullptr, 0.0};
    return checkMotion(from, to, last_valid);
  }

  bool checkMotion(const ob::State* from, const ob::State* to,
                   std::pair<ob::State*, double>& last_valid) const override {
    const auto& space = *si_->getStateSpace()->as<ob::ConstrainedStateSpace>();
    std::vector<ob::State*> walk;
    const bool walked =
        space.getConstraint()->isSatisfied(to) && space.discreteGeodesic(from, to, true, &walk);
    const auto outside = std::find_if(walk.begin(), walk.end(), [this](const ob::State* state) {
      return !si_->satisfiesBounds(state);
    });
    const bool finished = walked && outside == walk.end() && si_->satisfiesBounds(to);
    std::vector<const ob::State*> points{from};
    if (outside - walk.begin() > 1) {
      points.insert(points.end(), walk.begin() + 1, outside);
    }
    if (finished) {
      points.push_back(to);
    }
    std::size_t reached = 0;
    double traveled = 0.0;
    bool passes = true;
    for (std::size_t at = 1; passes && at < points.size(); ++at) {
      if (at + 1 < points.size() && on_stretch(points[reached], points[at], points[at + 1])) {
        continue;
      }
      double unused = 0.0;
      passes = check(points[reached], points[at], unused);
      if (passes) {
        traveled += space.distance(points[reached], points[at]);
        reached = at;
      }
    }
    const bool valid = finished && passes;
    if (!valid) {
      const double whole = traveled + space.distance(points[reached], to);
      if (last_valid.first != nullptr) {
        space.copyState(last_valid.first, points[reached]);
      }
      last_valid.second = whole > 0.0 ? traveled / whole : 0.0;
    }
    for (ob::State* state : walk) {
      space.freeState(state);
    }
    return count(valid);
  }

 private:
  // Whether `at` lies on the straight segment from `first` to `last`, to
  // `stretch_tolerance`.
  bool on_stretch(const ob::State* first, const ob::State* at, const ob::State* last) const {
    const auto vector_of = [this](const ob::State* state) {
      return Eigen::Map<const Eigen::VectorXd>(values(state, true),
                                               static_cast<Eigen::Index>(dimension_));
    };
    const Eigen::VectorXd line = vector_of(last) - vector_of(first);
    const Eigen::VectorXd along = vector_of(at) - vector_of(first);
    const double squared = line.squaredNorm();
    if (squared == 0.0) {
      return false;
    }
    const double share = along.dot(line) / squared;
    return share >= 0.0 && share <= 1.0 && (along - share * line).norm() <= stretch_tolerance;
  }
};

// The caller's constraint: its function and its Jacobian, which the caller writes
// row by row. Each call hands the caller a buffer of NaN, so a value that a failed
// call leaves unwritten is NaN, which OMPL's projection and its test of the
// constraint take as off the constraint.
class CallerConstraint : public ob::Constraint {
 public:
  CallerConstraint(unsigned int dimension, unsigned int codimension, double tolerance,
                   unsigned int iterations, const Callbacks& callbacks)
      : ob::Constraint(dimension, codimension, tolerance), callbacks_(callbacks) {
    setMaxIterations(iterations);
  }

  void function(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::Ref<Eigen::VectorXd> out) const override {
    out.setConstant(std::numeric_limits<double>::quiet_NaN());
    callbacks_.constraint_function(rust::Slice<const double>(x.data(), x.size()),
                                   rust::Slice<double>(out.data(), out.size()));
  }

  void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::Ref<Eigen::MatrixXd> out) const override {
    const auto rows = static_cast<std::size_t>(out.rows());
    const auto columns = static_cast<std::size_t>(out.cols());
    std::vector<double> by_rows(rows * columns, std::numeric_limits<double>::quiet_NaN());
    callbacks_.constraint_jacobian(rust::Slice<const double>(x.data(), x.size()),
                                   rust::Slice<double>(by_rows.data(), by_rows.size()));
    for (std::size_t r = 0; r < rows; ++r) {
      for (std::size_t c = 0; c < columns; ++c) {
        out(r, c) = by_rows[r * columns + c];
      }
    }
  }

 private:
  const Callbacks& callbacks_;
};

// The questions whether to stop in a round of a roadmap's expansion, and in a round of its
// growth, twice as many, as OMPL's PRM grows its roadmap for 0.4 s and expands it for
// 0.2 s in turn.
constexpr unsigned int expansion_round = 500;
constexpr unsigned int growth_round = 2 * expansion_round;

// The states of a random bounce motion of a roadmap's expansion: OMPL's
// MAX_RANDOM_BOUNCE_STEPS, which PRM.cpp keeps to itself.
constexpr std::size_t bounce_steps = 5;

// OMPL's PRM or PRM*, `Roadmap`, whose roadmap and path depend on the plan's questions
// whether to stop and on nothing else. Its solve is PRM::solve with five changes. It
// takes every goal into the roadmap after the start, before its search thread starts,
// the caller's sampled goals until their sampling ends, where PRM's search thread takes
// each goal after the first in a search of its own. It grows and expands the roadmap in
// rounds of questions, where PRM's rounds are timed. Once the planner has ended, it
// searches the finished roadmap, unless its search thread found a path that ended the
// planner, as PRM's first path does. It sets the best cost to infinite before its search
// thread starts, where PRM::constructRoadmap sets it after PRM::solve has started that
// thread, which may have written it by then. And it leaves out PRM::solve's messages, and
// its refusals of a goal that is not a sampleable region and of one that cannot be
// sampled: the bridge's goals are sampleable regions, and goals that give the roadmap
// none answer an invalid goal, or no path once the plan's bounds have ended it.
template <typename Roadmap>
class RepeatableRoadmap : public Roadmap {
 public:
  using Roadmap::Roadmap;

  ob::PlannerStatus solve(const ob::PlannerTerminationCondition& ptc) override {
    this->checkValidity();
    while (const ob::State* start = this->pis_.nextStart()) {
      this->startM_.push_back(this->addMilestone(this->si_->cloneState(start)));
    }
    if (this->startM_.empty()) {
      return ob::PlannerStatus::INVALID_START;
    }
    // Each call samples the next goal, and answers it when it passes the state check.
    while (this->pis_.haveMoreGoalStates()) {
      if (const ob::State* goal = this->pis_.nextGoal()) {
        this->goalM_.push_back(this->addMilestone(this->si_->cloneState(goal)));
      }
    }
    if (this->goalM_.empty()) {
      return ob::PlannerStatus::INVALID_GOAL;
    }

    // The search thread updates the best cost, which starts infinite.
    this->bestCost_ = this->opt_->infiniteCost();
    this->addedNewSolution_ = false;
    ob::PathPtr solution;
    std::thread search([this, &ptc, &solution] { this->checkForSolution(ptc, solution); });
    construct_in_rounds(ob::PlannerTerminationCondition(
        [this, &ptc] { return ptc || this->addedNewSolution(); }));
    search.join();
    if (!this->addedNewSolution()) {
      this->maybeConstructSolution(this->startM_, this->goalM_, solution);
    }

    if (solution) {
      ob::PlannerSolution found(solution);
      found.setPlannerName(this->getName());
      found.setOptimized(this->opt_, this->bestCost_, this->addedNewSolution());
      this->pdef_->addSolutionPath(found);
      return ob::PlannerStatus::EXACT_SOLUTION;
    }
    const ob::Cost to_go =
        this->constructApproximateSolution(this->startM_, this->goalM_, solution);
    if (!this->opt_->isFinite(to_go)) {
      return ob::PlannerStatus::TIMEOUT;
    }
    this->pdef_->addSolutionPath(solution, true, to_go.value(), this->getName());
    return ob::PlannerStatus::APPROXIMATE_SOLUTION;
  }

 private:
  // PRM::constructRoadmap, whose rounds of growth and of expansion each end at their
  // question past `growth_round` and `expansion_round`.
  void construct_in_rounds(const ob::PlannerTerminationCondition& ptc) {
    if (!this->sampler_) {
      this->sampler_ = this->si_->allocValidStateSampler();
    }
    if (!this->simpleSampler_) {
      this->simpleSampler_ = this->si_->allocStateSampler();
    }
    std::vector<ob::State*> work(bounce_steps);
    this->si_->allocStates(work);
    for (bool grow = true; !ptc(); grow = !grow) {
      ob::IterationTerminationCondition round(grow ? growth_round : expansion_round);
      const auto ended = ob::plannerOrTerminationCondition(ptc, round);
      if (grow) {
        this->growRoadmap(ended, work[0]);
      } else {
        this->expandRoadmap(ended, work);
      }
    }
    this->si_->freeStates(work);
  }
};

ob::PlannerPtr planner_of(Planner planner, const ob::SpaceInformationPtr& si) {
  switch (planner) {
    case Planner::RrtConnect:
      return std::make_shared<og::RRTConnect>(si);
    case Planner::Rrt:
      return std::make_shared<og::RRT>(si);
    case Planner::RrtStar:
      return std::make_shared<og::RRTstar>(si);
    case Planner::BiTrrt:
      return std::make_shared<og::BiTRRT>(si);
    case Planner::Kpiece1:
      return std::make_shared<og::KPIECE1>(si);
    case Planner::Bkpiece1:
      return std::make_shared<og::BKPIECE1>(si);
    case Planner::Lbkpiece1:
      return std::make_shared<og::LBKPIECE1>(si);
    case Planner::Est:
      return std::make_shared<og::EST>(si);
    case Planner::Sbl:
      return std::make_shared<og::SBL>(si);
    case Planner::Prm:
      return std::make_shared<RepeatableRoadmap<og::PRM>>(si);
    case Planner::PrmStar:
      return std::make_shared<RepeatableRoadmap<og::PRMstar>>(si);
  }
  throw std::invalid_argument("no planner has the number " +
                              std::to_string(static_cast<unsigned>(planner)));
}

// A state of `space` holding `values`.
ob::ScopedState<> state_of(const ob::StateSpacePtr& space, const double* values,
                           std::size_t dimension, bool constrained) {
  ob::ScopedState<> state(space);
  if (constrained) {
    state->as<ob::ConstrainedStateSpace::StateType>()->copy(
        Eigen::Map<const Eigen::VectorXd>(values, static_cast<Eigen::Index>(dimension)));
  } else {
    auto* real = state->as<ob::RealVectorStateSpace::StateType>();
    for (std::size_t i = 0; i < dimension; ++i) {
      real->values[i] = values[i];
    }
  }
  return state;
}

rust::Vec<double> flattened(const og::PathGeometric& path, std::size_t dimension,
                            bool constrained) {
  rust::Vec<double> out;
  out.reserve(path.getStateCount() * dimension);
  for (std::size_t at = 0; at < path.getStateCount(); ++at) {
    const double* v = values(path.getState(static_cast<unsigned int>(at)), constrained);
    for (std::size_t i = 0; i < dimension; ++i) {
      out.push_back(v[i]);
    }
  }
  return out;
}

Status status_of(ob::PlannerStatus status) {
  switch (ob::PlannerStatus::StatusType(status)) {
    case ob::PlannerStatus::EXACT_SOLUTION:
      return Status::Exact;
    case ob::PlannerStatus::APPROXIMATE_SOLUTION:
      return Status::Approximate;
    case ob::PlannerStatus::TIMEOUT:
      return Status::NotSolved;
    case ob::PlannerStatus::INVALID_START:
      return Status::InvalidStart;
    case ob::PlannerStatus::INVALID_GOAL:
      return Status::InvalidGoal;
    default:
      return Status::Other;
  }
}

// The answer of a plan whose planner answered `status` and was ended by `ended_by`,
// before the plan writes a path into it.
PlanResult answer_of(ob::PlannerStatus status, Ending ended_by) {
  PlanResult result;
  result.status = status_of(status);
  result.status_name = status.asString();
  result.ended_by = ended_by;
  return result;
}

// What ends a plan's planners and its simplifier at their next question whether to stop,
// whatever their bounds: a failed check, then the caller's stop.
std::optional<Ending> interruption(const Callbacks& callbacks) {
  if (callbacks.failed()) {
    return Ending::Failure;
  }
  if (callbacks.stopped()) {
    return Ending::Stopped;
  }
  return std::nullopt;
}

// What keeps a step of a plan from starting, a batch of its attempts or its
// simplification: an interruption, then the passing of `deadline`.
std::optional<Ending> ended_before(const Callbacks& callbacks, Clock::time_point deadline) {
  if (const std::optional<Ending> interrupted = interruption(callbacks)) {
    return interrupted;
  }
  if (Clock::now() >= deadline) {
    return Ending::TimeLimit;
  }
  return std::nullopt;
}

// Whether `state` passes the state check of `si` and lies on `constraint`, when the plan
// has one.
bool passes(const ob::SpaceInformation& si, const CallerConstraint* constraint,
            const ob::State* state) {
  return si.isValid(state) && (constraint == nullptr || constraint->isSatisfied(state));
}

// The goals of an attempt that the caller samples while the attempt plans, as MoveIt's
// ConstrainedGoalSampler, an OMPL GoalLazySamples, samples a request's goals beside its
// planner (moveit2 2.15.2 constrained_goal_sampler.cpp), here on the thread whose question
// asks for one, so an attempt's goals and its path depend on its questions alone. A
// planner asks through maxSampleCount, which canSample, PlannerInputStates::nextGoal and
// haveMoreGoalStates call: once the planner has been handed every goal found, a question
// samples once, and before the first goal it samples until one is found or no sample may
// start, as a planner waits for its first goal (OMPL 2.0.2 PlannerInputStates::nextGoal).
// With `until_found`, for a planner that takes its goals before it plans, each question
// samples so.
// A goal found before `deadline` joins the goals when it passes the state check and lies
// on the constraint. The sampling ends when the caller's sampling ends, and no sample
// starts at an interruption or once `deadline` has passed. Until the sampling ends the
// goals could sample, so a planner left waiting for its first goal asks its termination
// condition, which ends it. The lock serves the threads on which PRM and PRM* search their
// roadmap.
class SampledGoals : public ob::GoalStates {
 public:
  SampledGoals(const ob::SpaceInformationPtr& si, const Callbacks& callbacks,
               std::uint32_t attempt, std::size_t dimension, bool constrained, bool until_found,
               std::shared_ptr<CallerConstraint> constraint, Clock::time_point deadline)
      : ob::GoalStates(si),
        callbacks_(callbacks),
        attempt_(attempt),
        dimension_(dimension),
        constrained_(constrained),
        until_found_(until_found),
        constraint_(std::move(constraint)),
        deadline_(deadline),
        values_(dimension) {}

  unsigned int maxSampleCount() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (const std::size_t held = getStateCount(); handed_ >= held) {
      do {
        sample();
      } while ((until_found_ || held == 0) && getStateCount() == held && may_start());
    }
    return static_cast<unsigned int>(getStateCount());
  }

  bool couldSample() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return !ended_;
  }

  void sampleGoal(ob::State* state) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    ob::GoalStates::sampleGoal(state);
    ++handed_;
  }

  double distanceGoal(const ob::State* state) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return ob::GoalStates::distanceGoal(state);
  }

  // Ends the sampling, as MoveIt's postSolve stops its sampling thread: the simplifier, which
  // reads a plan's goals, takes those found by then.
  void stop_sampling() {
    std::lock_guard<std::mutex> lock(mutex_);
    ended_ = true;
  }

  // The goals found, flattened.
  std::vector<double> found() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<double> flat;
    flat.reserve(getStateCount() * dimension_);
    for (std::size_t at = 0; at < getStateCount(); ++at) {
      const double* goal = values(getState(static_cast<unsigned int>(at)), constrained_);
      flat.insert(flat.end(), goal, goal + dimension_);
    }
    return flat;
  }

  // Takes the flattened `goals`, found by the attempts of the plan.
  void take(const std::vector<double>& goals) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t at = 0; at < goals.size(); at += dimension_) {
      addState(state_of(si_->getStateSpace(), goals.data() + at, dimension_, constrained_).get());
    }
  }

 private:
  // Whether a sample may start.
  bool may_start() const {
    return !ended_ && !interruption(callbacks_) && Clock::now() < deadline_;
  }

  // One sample of the caller's sampling, when one may start, which adds the goal it finds.
  // OMPL asks for goals through const functions, so the goals grow through a cast.
  void sample() const {
    if (!may_start()) {
      return;
    }
    switch (callbacks_.sample_goal(attempt_, rust::Slice<double>(values_.data(), dimension_))) {
      case GoalSample::Found: {
        const auto goal = state_of(si_->getStateSpace(), values_.data(), dimension_, constrained_);
        if (Clock::now() < deadline_ && passes(*si_, constraint_.get(), goal.get())) {
          const_cast<SampledGoals*>(this)->ob::GoalStates::addState(goal.get());
        }
        return;
      }
      case GoalSample::Missed:
        return;
      case GoalSample::Ended:
        ended_ = true;
        return;
    }
  }

  const Callbacks& callbacks_;
  std::uint32_t attempt_;
  std::size_t dimension_;
  bool constrained_;
  bool until_found_;
  std::shared_ptr<CallerConstraint> constraint_;
  Clock::time_point deadline_;
  mutable std::mutex mutex_;
  mutable std::vector<double> values_;
  // The goals handed to the planner, repeats among them, counted over the goals found.
  mutable std::size_t handed_ = 0;
  mutable bool ended_ = false;
};

// A termination condition that ends the planner or the simplifier it is given to at an
// interruption, at the first question after the thread that built it has asked it
// `iterations` times, or once `deadline` has passed, tested in that order, and
// remembers what ended it first. Another thread, such as the one on which PRM and PRM*
// search their roadmap for a solution, adds nothing to the count, so the count does not
// depend on how the plan's threads are scheduled.
class Bounds {
 public:
  Bounds(const Callbacks& callbacks, std::uint64_t iterations, Clock::time_point deadline)
      : callbacks_(callbacks),
        iterations_(iterations),
        deadline_(deadline),
        counting_thread_(std::this_thread::get_id()) {}

  ob::PlannerTerminationCondition condition() {
    return ob::PlannerTerminationCondition([this] { return reached(); });
  }

  Ending ended_by() const { return ended_by_.load(); }

 private:
  bool reached() {
    const std::optional<Ending> ending = ending_now();
    if (!ending) {
      return false;
    }
    Ending unended = Ending::Returned;
    ended_by_.compare_exchange_strong(unended, *ending);
    return true;
  }

  std::optional<Ending> ending_now() {
    if (const std::optional<Ending> interrupted = interruption(callbacks_)) {
      return interrupted;
    }
    if (at_iteration_bound()) {
      return Ending::IterationBound;
    }
    if (Clock::now() >= deadline_) {
      return Ending::TimeLimit;
    }
    return std::nullopt;
  }

  // Whether the iteration bound ends the question being asked. The counting thread's
  // question past its `iterations`th ends it, and another thread's question once the
  // counting thread has been ended by the bound.
  bool at_iteration_bound() {
    if (std::this_thread::get_id() == counting_thread_) {
      return asked_.fetch_add(1) >= iterations_;
    }
    return asked_.load() > iterations_;
  }

  const Callbacks& callbacks_;
  std::uint64_t iterations_;
  Clock::time_point deadline_;
  std::thread::id counting_thread_;
  std::atomic<std::uint64_t> asked_{0};
  std::atomic<Ending> ended_by_{Ending::Returned};
};

// The instant `seconds` after `from`, or the end of the clock when that lies beyond it.
Clock::time_point later_by(Clock::time_point from, double seconds) {
  const auto limit = std::chrono::duration<double>(seconds);
  if (limit >= Clock::time_point::max() - from) {
    return Clock::time_point::max();
  }
  return from + std::chrono::duration_cast<Clock::duration>(limit);
}

// `path` as a plan answers it: a constrained path walked along the constraint, as
// OMPL's demos of constrained planning and MoveIt interpolate one, in steps no longer
// than the space's spacing.
og::PathGeometric answered(og::PathGeometric path, bool constrained) {
  if (constrained) {
    path.interpolate();
  }
  return path;
}

// A request's problem: a setup of its state space, the space's information with the
// caller's checks, its start, its goals, the given ones that pass the state check and lie
// on the constraint or the caller's sampled goals, and its planner.
// The caller's check of a motion of `si`'s space, walked along the constraint when the
// space is `constrained`: the planner's, or the simplifier's when `simplifying`.
ob::MotionValidatorPtr motion_check(const ob::SpaceInformationPtr& si, const Callbacks& callbacks,
                                    std::size_t dimension, bool constrained, bool simplifying) {
  if (constrained) {
    return std::make_shared<WalkedMotionCheck>(si, callbacks, dimension, simplifying);
  }
  return std::make_shared<MotionCheck>(si, callbacks, dimension, simplifying);
}

struct Problem {
  og::SimpleSetupPtr setup;
  std::size_t dimension;
  bool constrained;
};

// The sampled goals of `problem`; none when its goals were given.
std::shared_ptr<SampledGoals> sampled_goals_of(const Problem& problem) {
  return std::dynamic_pointer_cast<SampledGoals>(problem.setup->getGoal());
}

// The problem of attempt `attempt` of `request`, built on the calling thread once its seed
// generator takes the attempt's seed, so that seed seeds every random number generator of
// the problem, the planner's, the samplers' and the simplifier's, in the order of the
// problem's construction; or the status that refuses the request's start, when it fails
// the state check or lies off the constraint, or its every given goal. Its sampled goals
// sample until `deadline`.
// A real vector space measured as MoveIt measures a joint group: the distance between two
// states is the sum of each dimension's absolute change, and the space's extent the sum of
// the dimensions' ranges (moveit2 2.15.2 joint_model_group.cpp:455-476).
class JointGroupSpace : public ob::RealVectorStateSpace {
 public:
  using ob::RealVectorStateSpace::RealVectorStateSpace;

  double distance(const ob::State* from, const ob::State* to) const override {
    const double* a = from->as<StateType>()->values;
    const double* b = to->as<StateType>()->values;
    double sum = 0.0;
    for (unsigned int i = 0; i < getDimension(); ++i) {
      sum += std::fabs(a[i] - b[i]);
    }
    return sum;
  }

  double getMaximumExtent() const override {
    double sum = 0.0;
    for (unsigned int i = 0; i < getDimension(); ++i) {
      sum += bounds_.high[i] - bounds_.low[i];
    }
    return sum;
  }
};

std::variant<Problem, ob::PlannerStatus::StatusType> posed(const PlanRequest& request,
                                                           const Callbacks& callbacks,
                                                           std::uint32_t attempt,
                                                           Clock::time_point deadline) {
  ompl::RNG::setThreadSeed(attempt_seed(request.seed, attempt));
  const std::size_t dimension = request.low.size();
  auto real = std::make_shared<JointGroupSpace>(dimension);
  ob::RealVectorBounds bounds(dimension);
  for (std::size_t i = 0; i < dimension; ++i) {
    bounds.setLow(i, request.low[i]);
    bounds.setHigh(i, request.high[i]);
  }
  real->setBounds(bounds);

  const bool constrained = request.codimension > 0;
  ob::StateSpacePtr space;
  ob::SpaceInformationPtr si;
  std::shared_ptr<CallerConstraint> constraint;
  if (constrained) {
    constraint = std::make_shared<CallerConstraint>(static_cast<unsigned int>(dimension),
                                                    request.codimension, request.tolerance,
                                                    request.projection_iterations, callbacks);
    auto projected = std::make_shared<ob::ProjectedStateSpace>(real, constraint);
    projected->setDelta(request.constrained_spacing / projected->getLambda());
    space = projected;
    si = std::make_shared<ob::ConstrainedSpaceInformation>(space);
    si->setMotionValidator(motion_check(si, callbacks, dimension, true, false));
  } else {
    space = real;
    si = std::make_shared<ob::SpaceInformation>(space);
    si->setMotionValidator(motion_check(si, callbacks, dimension, false, false));
  }
  si->setStateValidityChecker(std::make_shared<StateCheck>(si, callbacks, dimension, constrained));

  const auto start = state_of(space, request.start.data(), dimension, constrained);
  if (!passes(*si, constraint.get(), start.get())) {
    return ob::PlannerStatus::INVALID_START;
  }
  auto setup = std::make_shared<og::SimpleSetup>(si);
  ob::GoalPtr goals;
  if (request.sampled_goals) {
    const bool until_found = request.planner == Planner::Prm || request.planner == Planner::PrmStar;
    goals = std::make_shared<SampledGoals>(si, callbacks, attempt, dimension, constrained,
                                           until_found, constraint, deadline);
  } else {
    auto given = std::make_shared<ob::GoalStates>(si);
    for (std::size_t at = 0; at < request.goals.size(); at += dimension) {
      const auto goal = state_of(space, request.goals.data() + at, dimension, constrained);
      if (passes(*si, constraint.get(), goal.get())) {
        given->addState(goal);
      }
    }
    if (!given->hasStates()) {
      return ob::PlannerStatus::INVALID_GOAL;
    }
    goals = given;
  }

  setup->setStartState(start);
  setup->setGoal(goals);
  setup->setPlanner(planner_of(request.planner, si));
  setup->setup();
  return Problem{setup, dimension, constrained};
}

// What an attempt of a plan found: what its planner answered, what ended it, the path,
// flattened, once it found an exact one, and its sampled goals, flattened.
struct Attempt {
  ob::PlannerStatus status;
  Ending ended_by = Ending::Returned;
  rust::Vec<double> path;
  std::vector<double> goals;
};

// The attempt at `problem` on the calling thread until `deadline`, whose thread's
// questions alone count toward the iteration bound. Its path stays in the problem's
// setup, and its sampling ends with its planner. A planner left with no goal once the plan
// is interrupted or `deadline` has passed answers no path, as a planner that they end later
// does.
Attempt solved(const Problem& problem, const PlanRequest& request, const Callbacks& callbacks,
               Clock::time_point deadline) {
  Bounds bounds(callbacks, request.iterations, deadline);
  Attempt attempt{problem.setup->solve(bounds.condition())};
  attempt.ended_by = bounds.ended_by();
  if (const std::shared_ptr<SampledGoals> sampled = sampled_goals_of(problem)) {
    sampled->stop_sampling();
    attempt.goals = sampled->found();
  }
  if (attempt.status == ob::PlannerStatus::INVALID_GOAL) {
    if (const std::optional<Ending> ended = ended_before(callbacks, deadline)) {
      attempt.status = ob::PlannerStatus::TIMEOUT;
      attempt.ended_by = attempt.ended_by == Ending::Returned ? *ended : attempt.ended_by;
    }
  }
  return attempt;
}

// Attempt `attempt` at `request` until `deadline` on the calling thread, with its path; a
// request that its problem refuses plans nothing.
Attempt attempt_from(const PlanRequest& request, const Callbacks& callbacks,
                     std::uint32_t attempt, Clock::time_point deadline) {
  const auto problem = posed(request, callbacks, attempt, deadline);
  if (const auto* refused = std::get_if<ob::PlannerStatus::StatusType>(&problem)) {
    return Attempt{*refused};
  }
  const Problem& own = std::get<Problem>(problem);
  Attempt ended = solved(own, request, callbacks, deadline);
  if (status_of(ended.status) == Status::Exact && !callbacks.failed()) {
    ended.path = flattened(own.setup->getSolutionPath(), own.dimension, own.constrained);
  }
  return ended;
}

// The ending of a plan whose attempts ended by `a` and by `b`: the caller's stop over a
// failed check, over the time limit, over the iteration bound, over the planners' own
// return.
Ending prevailing(Ending a, Ending b) {
  for (const Ending ending :
       {Ending::Stopped, Ending::Failure, Ending::TimeLimit, Ending::IterationBound}) {
    if (a == ending || b == ending) {
      return ending;
    }
  }
  return Ending::Returned;
}

// What the attempts of a plan found: the paths of those that found an exact one, in
// attempt order; their sampled goals, flattened, in attempt order; the exact solution once
// one found it, and otherwise what attempt 0 answered, or a timeout when no attempt ran;
// and what ended them.
struct Attempts {
  std::vector<rust::Vec<double>> paths;
  std::vector<double> goals;
  ob::PlannerStatus status = ob::PlannerStatus::TIMEOUT;
  Ending ended_by = Ending::Returned;
};

// The attempts of `request` until `deadline`, `request.planning_threads` at once, each
// on a thread of its own from its own seed. A batch starts once the one before it has
// ended, unless the plan is interrupted or `deadline` has passed, which then ends the
// plan's attempts. An exception of an attempt is rethrown once its batch has ended.
Attempts attempts_of(const PlanRequest& request, const Callbacks& callbacks,
                     Clock::time_point deadline) {
  Attempts attempts;
  for (std::uint32_t first = 0; first < request.attempts;) {
    if (const std::optional<Ending> ended = ended_before(callbacks, deadline)) {
      attempts.ended_by = prevailing(attempts.ended_by, *ended);
      break;
    }
    const std::uint32_t size = std::min(request.planning_threads, request.attempts - first);
    std::vector<Attempt> batch(size);
    std::vector<std::exception_ptr> errors(size);
    std::vector<std::thread> threads;
    // Reserved, so that a thread that starts always joins; a thread that cannot start
    // ends the plan once the started ones have joined.
    threads.reserve(size);
    const auto join = [&threads] {
      for (std::thread& thread : threads) {
        thread.join();
      }
    };
    try {
      for (std::uint32_t k = 0; k < size; ++k) {
        const std::uint32_t attempt = first + k;
        threads.emplace_back([&request, &callbacks, &batch, &errors, k, attempt, deadline] {
          try {
            batch[k] = attempt_from(request, callbacks, attempt, deadline);
          } catch (...) {
            errors[k] = std::current_exception();
          }
        });
      }
    } catch (...) {
      join();
      throw;
    }
    join();
    for (const std::exception_ptr& error : errors) {
      if (error) {
        std::rethrow_exception(error);
      }
    }
    for (std::uint32_t k = 0; k < size; ++k) {
      Attempt& ended = batch[k];
      if (first + k == 0) {
        attempts.status = ended.status;
      }
      attempts.ended_by = prevailing(attempts.ended_by, ended.ended_by);
      attempts.goals.insert(attempts.goals.end(), ended.goals.begin(), ended.goals.end());
      if (!ended.path.empty()) {
        attempts.status = ob::PlannerStatus::EXACT_SOLUTION;
        attempts.paths.push_back(std::move(ended.path));
      }
    }
    first += size;
  }
  return attempts;
}

// The path of the flattened `values` in `problem`'s space.
og::PathGeometricPtr path_of(const Problem& problem, const rust::Vec<double>& values) {
  auto path = std::make_shared<og::PathGeometric>(problem.setup->getSpaceInformation());
  for (std::size_t at = 0; at < values.size(); at += problem.dimension) {
    path->append(state_of(problem.setup->getStateSpace(), values.data() + at, problem.dimension,
                          problem.constrained)
                     .get());
  }
  return path;
}

// The hybrid of `paths`, two or more paths of `problem`'s space, as OMPL's ParallelPlan
// joins the paths of its planners: each path recorded in turn and joined to each
// recorded before it where their states align, by the motions between aligned states
// that pass the motion check from the later path's state, and the hybrid the shortest
// path through them all; none when OMPL finds none.
og::PathGeometricPtr hybrid_of(const Problem& problem,
                               const std::vector<og::PathGeometricPtr>& paths) {
  og::PathHybridization hybridization(problem.setup->getSpaceInformation());
  for (const og::PathGeometricPtr& path : paths) {
    hybridization.recordPath(path, false);
  }
  hybridization.computeHybridPath();
  return hybridization.getHybridPath();
}

// Writes `planned`, as the plan answers it, into `result`, and, when the request asks for
// it, its simplification. OMPL's simplifier, judging each motion by the caller's simplifier
// check, smooths and repairs the path it shortens, and says when the result may touch an
// invalid state; its result stands when it is valid
// and at most as long as the path. It starts unless the plan is interrupted, past
// `deadline` too, as MoveIt's `simplifySolution` starts with whatever time its plan
// left: an interruption keeps it from starting, and the plan answers that. Its first
// pass runs to its end whatever the termination condition answers: OMPL bounds that pass
// by counts of attempts, most of them the path's number of states. Each pass after it
// ends at the iteration bound, at an interruption or at `deadline`.
void write_path(PlanResult& result, const Problem& problem, const og::PathGeometric& planned,
                const PlanRequest& request, const Callbacks& callbacks,
                Clock::time_point deadline) {
  const og::PathGeometric path = answered(planned, problem.constrained);
  result.path = flattened(path, problem.dimension, problem.constrained);
  if (request.simplification_iterations == 0) {
    return;
  }
  og::PathGeometric simplified(planned);
  bool valid = false;
  if (const std::optional<Ending> interrupted = interruption(callbacks)) {
    result.simplification_ended_by = *interrupted;
  } else {
    const ob::SpaceInformationPtr& si = problem.setup->getSpaceInformation();
    si->setMotionValidator(
        motion_check(si, callbacks, problem.dimension, problem.constrained, true));
    Bounds simplification_bounds(callbacks, request.simplification_iterations, deadline);
    valid = problem.setup->getPathSimplifier()->simplify(simplified,
                                                          simplification_bounds.condition());
    result.simplification_ended_by = simplification_bounds.ended_by();
  }
  const og::PathGeometric simplified_path = answered(simplified, problem.constrained);
  const bool better = valid && simplified_path.length() <= path.length();
  result.simplified =
      flattened(better ? simplified_path : path, problem.dimension, problem.constrained);
}

// A plan of one attempt, on the calling thread from the request's seed: its problem
// plans, and simplifies the path it finds.
PlanResult plan_once(const PlanRequest& request, const Callbacks& callbacks,
                     Clock::time_point deadline, Clock::time_point simplification_deadline) {
  const auto problem = posed(request, callbacks, 0, deadline);
  if (const auto* refused = std::get_if<ob::PlannerStatus::StatusType>(&problem)) {
    return answer_of(*refused, Ending::Returned);
  }
  const Problem& one = std::get<Problem>(problem);
  const Attempt attempt = solved(one, request, callbacks, deadline);
  PlanResult result = answer_of(attempt.status, attempt.ended_by);
  if (result.status != Status::Exact || callbacks.failed()) {
    return result;
  }
  write_path(result, one, one.setup->getSolutionPath(), request, callbacks,
             simplification_deadline);
  return result;
}

// A plan of several attempts. Its problem, posed on the calling thread as that of attempt
// `attempts`, which plans nothing and so samples no goal, refuses the request's start or
// its given goals before any attempt starts, takes the goals the attempts sample, and joins
// and simplifies what the attempts find. The paths of two or more join into
// their hybrid, unless the plan is stopped, and the plan keeps the shortest of the paths
// and the hybrid, the first of the equally short in attempt order, the hybrid last; the
// hybrid when it passes the checks, the state check of its start and the motion check of
// each motion in its order, since OMPL joins two paths by a motion checked in one
// direction and may take it in the other.
PlanResult plan_attempts(const PlanRequest& request, const Callbacks& callbacks,
                         Clock::time_point deadline, Clock::time_point simplification_deadline) {
  const auto problem = posed(request, callbacks, request.attempts, deadline);
  if (const auto* refused = std::get_if<ob::PlannerStatus::StatusType>(&problem)) {
    return answer_of(*refused, Ending::Returned);
  }
  const Problem& joining = std::get<Problem>(problem);
  const std::shared_ptr<SampledGoals> sampled = sampled_goals_of(joining);
  if (sampled) {
    sampled->stop_sampling();
  }
  const Attempts attempts = attempts_of(request, callbacks, deadline);
  if (sampled) {
    sampled->take(attempts.goals);
  }
  PlanResult result = answer_of(attempts.status, attempts.ended_by);
  if (attempts.paths.empty() || callbacks.failed()) {
    return result;
  }
  std::vector<og::PathGeometricPtr> candidates;
  for (const rust::Vec<double>& path : attempts.paths) {
    candidates.push_back(path_of(joining, path));
  }
  if (candidates.size() > 1 && !callbacks.stopped()) {
    if (const og::PathGeometricPtr hybrid = hybrid_of(joining, candidates);
        hybrid && hybrid->check()) {
      candidates.push_back(hybrid);
    }
  }
  const auto shortest = std::min_element(
      candidates.begin(), candidates.end(),
      [](const og::PathGeometricPtr& a, const og::PathGeometricPtr& b) {
        return a->length() < b->length();
      });
  write_path(result, joining, **shortest, request, callbacks, simplification_deadline);
  return result;
}

}  // namespace

PlanResult plan(const PlanRequest& request, const Callbacks& callbacks) {
  static std::once_flag log_level;
  std::call_once(log_level, [] { ompl::msg::setLogLevel(ompl::msg::LOG_WARN); });
  const Clock::time_point deadline = later_by(Clock::now(), request.time_limit_s);
  const Clock::time_point simplification_deadline =
      later_by(deadline, request.simplification_time_s);
  if (request.attempts == 1) {
    return plan_once(request, callbacks, deadline, simplification_deadline);
  }
  return plan_attempts(request, callbacks, deadline, simplification_deadline);
}

}  // namespace ompl_bridge
