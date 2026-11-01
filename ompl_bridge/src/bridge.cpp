#include "ompl_bridge/src/bridge.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
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

// OMPL seeds each of its random number generators from one seed generator of the
// process. A plan holds this lock from its seed to its last draw, so its generators
// take their seeds in the order of its own construction. A plan waits for the lock
// until its deadline.
std::timed_mutex plan_lock;

// Reseeds OMPL's seed generator. RNG::setSeed reseeds it however many seeds it has
// handed out, and reports it as an error once it has handed out any; each plan
// reseeds, so the report is silenced.
void reseed(std::uint32_t seed) {
  const auto level = ompl::msg::getLogLevel();
  ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(level);
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

// The caller's check of the straight motion between two states of a real vector
// space.
class MotionCheck : public ob::MotionValidator {
 public:
  MotionCheck(const ob::SpaceInformationPtr& si, const Callbacks& callbacks, std::size_t dimension)
      : ob::MotionValidator(si), callbacks_(callbacks), dimension_(dimension) {}

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

 private:
  bool check(const ob::State* from, const ob::State* to, double& last_valid) const {
    return callbacks_.motion_valid(rust::Slice<const double>(values(from, false), dimension_),
                                   rust::Slice<const double>(values(to, false), dimension_),
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

  const Callbacks& callbacks_;
  std::size_t dimension_;
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
// where PRM's search thread takes each goal after the first in a search of its own. It
// grows and expands the roadmap in rounds of questions, where PRM's rounds are timed.
// Once the planner has ended, it searches the finished roadmap, unless its search thread
// found a path that ended the planner, as PRM's first path does. It sets the best cost to
// infinite before its search thread starts, where PRM::constructRoadmap sets it after
// PRM::solve has started that thread, which may have written it by then. And it leaves
// out PRM::solve's messages, and its refusals of a goal that is not a sampleable region
// and of one that cannot be sampled, which the bridge's GoalStates of at least one state
// never meets.
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

// The answer to a plan that ends before its planner starts.
PlanResult ended_before_planning(ob::PlannerStatus::StatusType status, Ending ended_by) {
  PlanResult result;
  result.status = status_of(status);
  result.status_name = ob::PlannerStatus(status).asString();
  result.ended_by = ended_by;
  return result;
}

// A termination condition that ends the planner or the simplifier it is given to once
// the caller's checks fail, at the first question after the thread that built it has
// asked it `iterations` times, or once `deadline` has passed, tested in that order, and
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
    if (callbacks_.failed()) {
      return Ending::Failure;
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

// The instant `seconds` from now, or the end of the clock when that lies beyond it.
Clock::time_point deadline_after(double seconds) {
  const auto now = Clock::now();
  const auto limit = std::chrono::duration<double>(seconds);
  if (limit >= Clock::time_point::max() - now) {
    return Clock::time_point::max();
  }
  return now + std::chrono::duration_cast<Clock::duration>(limit);
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

}  // namespace

PlanResult plan(const PlanRequest& request, const Callbacks& callbacks) {
  const Clock::time_point deadline = deadline_after(request.time_limit_s);
  const std::unique_lock<std::timed_mutex> lock(plan_lock, deadline);
  if (!lock.owns_lock()) {
    return ended_before_planning(ob::PlannerStatus::TIMEOUT, Ending::TimeLimit);
  }
  ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
  reseed(request.seed);

  const std::size_t dimension = request.low.size();
  auto real = std::make_shared<ob::RealVectorStateSpace>(dimension);
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
  } else {
    space = real;
    si = std::make_shared<ob::SpaceInformation>(space);
    si->setMotionValidator(std::make_shared<MotionCheck>(si, callbacks, dimension));
  }
  si->setStateValidityChecker(std::make_shared<StateCheck>(si, callbacks, dimension, constrained));

  // Whether `state` passes the state check and lies on the constraint.
  const auto passes = [&](const ob::State* state) {
    return si->isValid(state) && (!constraint || constraint->isSatisfied(state));
  };
  const auto start = state_of(space, request.start.data(), dimension, constrained);
  if (!passes(start.get())) {
    return ended_before_planning(ob::PlannerStatus::INVALID_START, Ending::Returned);
  }
  auto goals = std::make_shared<ob::GoalStates>(si);
  for (std::size_t at = 0; at < request.goals.size(); at += dimension) {
    const auto goal = state_of(space, request.goals.data() + at, dimension, constrained);
    if (passes(goal.get())) {
      goals->addState(goal);
    }
  }
  if (!goals->hasStates()) {
    return ended_before_planning(ob::PlannerStatus::INVALID_GOAL, Ending::Returned);
  }

  og::SimpleSetup setup(si);
  setup.setStartState(start);
  setup.setGoal(goals);
  setup.setPlanner(planner_of(request.planner, si));
  setup.setup();

  Bounds plan_bounds(callbacks, request.iterations, deadline);
  const ob::PlannerStatus status = setup.solve(plan_bounds.condition());

  PlanResult result;
  result.status = status_of(status);
  result.status_name = status.asString();
  result.ended_by = plan_bounds.ended_by();
  if (result.status != Status::Exact || callbacks.failed()) {
    return result;
  }
  const og::PathGeometric& planned = setup.getSolutionPath();
  const og::PathGeometric path = answered(planned, constrained);
  result.path = flattened(path, dimension, constrained);
  if (request.simplification_iterations == 0) {
    return result;
  }
  // OMPL's simplifier smooths and repairs the path it shortens, and says when the
  // result may touch an invalid state; its result stands when it is valid and at most
  // as long as the path. It starts only before the deadline: past it, the plan answers
  // that the deadline ended the simplification. Its first pass runs to its end whatever
  // the termination condition answers: OMPL bounds that pass by counts of attempts,
  // most of them the path's number of states. Each pass after it ends at the iteration
  // bound or the deadline.
  og::PathGeometric simplified(planned);
  bool valid = false;
  result.simplification_ended_by = Ending::TimeLimit;
  if (Clock::now() < deadline) {
    Bounds simplification_bounds(callbacks, request.simplification_iterations, deadline);
    valid = setup.getPathSimplifier()->simplify(simplified, simplification_bounds.condition());
    result.simplification_ended_by = simplification_bounds.ended_by();
  }
  const og::PathGeometric simplified_path = answered(simplified, constrained);
  const bool better = valid && simplified_path.length() <= path.length();
  result.simplified = flattened(better ? simplified_path : path, dimension, constrained);
  return result;
}

}  // namespace ompl_bridge
