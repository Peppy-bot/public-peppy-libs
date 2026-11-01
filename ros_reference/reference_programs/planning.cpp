#include "planning.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
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

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ros_reference {

namespace {

// A plan's time limit: its iteration bound ends it first.
constexpr double time_limit_s = 600.0;

// An axis-aligned box of the unit cube, open: a state on its surface is free.
struct Obstacle {
  std::vector<double> min;
  std::vector<double> max;

  bool holds(const double* state) const {
    for (std::size_t i = 0; i < min.size(); ++i) {
      if (!(min[i] < state[i] && state[i] < max[i])) {
        return false;
      }
    }
    return true;
  }
};

struct World {
  std::size_t dimension;
  std::vector<Obstacle> obstacles;
  std::vector<double> start;
  std::vector<double> goal;
  double motion_step;

  bool free(const double* state) const {
    for (const auto& obstacle : obstacles) {
      if (obstacle.holds(state)) {
        return false;
      }
    }
    return true;
  }
};

const double* values(const ob::State* state) {
  return state->as<ob::RealVectorStateSpace::StateType>()->values;
}

class StateCheck : public ob::StateValidityChecker {
 public:
  StateCheck(const ob::SpaceInformationPtr& si, const World& world)
      : ob::StateValidityChecker(si), world_(world) {}

  bool isValid(const ob::State* state) const override { return world_.free(values(state)); }

 private:
  const World& world_;
};

// The straight motion between two states walked in steps of the world's motion step,
// each state judged free or not: the walk of the parity test's world.
class MotionCheck : public ob::MotionValidator {
 public:
  MotionCheck(const ob::SpaceInformationPtr& si, const World& world)
      : ob::MotionValidator(si), world_(world) {}

  bool checkMotion(const ob::State* from, const ob::State* to) const override {
    double last_valid = 0.0;
    return count(walk(values(from), values(to), last_valid));
  }

  bool checkMotion(const ob::State* from, const ob::State* to,
                   std::pair<ob::State*, double>& last_valid) const override {
    double fraction = 0.0;
    const bool valid = walk(values(from), values(to), fraction);
    if (!valid) {
      if (last_valid.first != nullptr) {
        si_->getStateSpace()->interpolate(from, to, fraction, last_valid.first);
      }
      last_valid.second = fraction;
    }
    return count(valid);
  }

 private:
  bool walk(const double* from, const double* to, double& last_valid) const {
    const std::size_t n = world_.dimension;
    double squared = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      squared += (from[i] - to[i]) * (from[i] - to[i]);
    }
    const auto steps =
        static_cast<std::size_t>(std::max(std::ceil(std::sqrt(squared) / world_.motion_step), 1.0));
    std::vector<double> state(n);
    last_valid = 0.0;
    for (std::size_t k = 1; k <= steps; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(steps);
      for (std::size_t i = 0; i < n; ++i) {
        state[i] = from[i] + (to[i] - from[i]) * t;
      }
      if (!world_.free(state.data())) {
        return false;
      }
      last_valid = t;
    }
    return true;
  }

  bool count(bool valid) const {
    if (valid) {
      ++valid_;
    } else {
      ++invalid_;
    }
    return valid;
  }

  const World& world_;
};

// The questions whether to stop in a round of a roadmap's expansion, and in a round of its
// growth, twice as many, as ompl_bridge builds a roadmap.
constexpr unsigned int expansion_round = 500;
constexpr unsigned int growth_round = 2 * expansion_round;

// The states of a random bounce motion of a roadmap's expansion: OMPL's
// MAX_RANDOM_BOUNCE_STEPS, which PRM.cpp keeps to itself.
constexpr std::size_t bounce_steps = 5;

// OMPL's PRM or PRM*, `Roadmap`, as ompl_bridge plans with it: PRM::solve, taking every
// goal into the roadmap after the start, before its search thread starts, growing and
// expanding the roadmap in rounds of questions whether to stop, and searching the
// finished roadmap once the planner has ended, unless its search thread found a path
// that ended the planner.
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

ob::PlannerPtr planner_named(const std::string& name, const ob::SpaceInformationPtr& si) {
  if (name == "RRTConnect") return std::make_shared<og::RRTConnect>(si);
  if (name == "RRT") return std::make_shared<og::RRT>(si);
  if (name == "RRTstar") return std::make_shared<og::RRTstar>(si);
  if (name == "BiTRRT") return std::make_shared<og::BiTRRT>(si);
  if (name == "KPIECE1") return std::make_shared<og::KPIECE1>(si);
  if (name == "BKPIECE1") return std::make_shared<og::BKPIECE1>(si);
  if (name == "LBKPIECE1") return std::make_shared<og::LBKPIECE1>(si);
  if (name == "EST") return std::make_shared<og::EST>(si);
  if (name == "SBL") return std::make_shared<og::SBL>(si);
  if (name == "PRM") return std::make_shared<RepeatableRoadmap<og::PRM>>(si);
  if (name == "PRMstar") return std::make_shared<RepeatableRoadmap<og::PRMstar>>(si);
  throw std::invalid_argument("no planner named " + name);
}

ob::ScopedState<> state_of(const ob::StateSpacePtr& space, const std::vector<double>& values) {
  ob::ScopedState<> state(space);
  for (std::size_t i = 0; i < values.size(); ++i) {
    state[i] = values[i];
  }
  return state;
}

// Ends a plan once the thread that built it has asked it `iterations` times, or at its
// deadline, and remembers what ended the plan first. Another thread, such as the one on
// which PRM and PRM* search their roadmap for a solution, adds nothing to the count, and
// sees the iteration bound once the counting thread has been ended by it, as ompl_bridge
// bounds a plan.
class Bounds {
 public:
  Bounds(std::uint64_t iterations, std::chrono::steady_clock::time_point deadline)
      : iterations_(iterations),
        deadline_(deadline),
        counting_thread_(std::this_thread::get_id()) {}

  ob::PlannerTerminationCondition condition() {
    return ob::PlannerTerminationCondition([this] { return reached(); });
  }

  std::string ended_by() const { return ended_by_.load(); }

 private:
  static constexpr const char* unended = "PlannerReturned";

  bool reached() {
    if (at_iteration_bound()) {
      return end("IterationBound");
    }
    if (std::chrono::steady_clock::now() >= deadline_) {
      return end("TimeLimit");
    }
    return false;
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

  bool end(const char* ending) {
    const char* expected = unended;
    ended_by_.compare_exchange_strong(expected, ending);
    return true;
  }

  std::uint64_t iterations_;
  std::chrono::steady_clock::time_point deadline_;
  std::thread::id counting_thread_;
  std::atomic<std::uint64_t> asked_{0};
  std::atomic<const char*> ended_by_{unended};
};

// The states of `path`, each as its values.
nlohmann::json states_of(const og::PathGeometric& path, std::size_t dimension) {
  auto out = nlohmann::json::array();
  for (std::size_t at = 0; at < path.getStateCount(); ++at) {
    const double* v = values(path.getState(static_cast<unsigned int>(at)));
    out.push_back(std::vector<double>(v, v + dimension));
  }
  return out;
}

World world_of(const nlohmann::json& world, double motion_step) {
  World out;
  out.start = world.at("start").get<std::vector<double>>();
  out.goal = world.at("goal").get<std::vector<double>>();
  out.dimension = out.start.size();
  out.motion_step = motion_step;
  for (const auto& obstacle : world.at("obstacles")) {
    out.obstacles.push_back({obstacle.at("min").get<std::vector<double>>(),
                             obstacle.at("max").get<std::vector<double>>()});
  }
  return out;
}

nlohmann::json echo_of(const World& world) {
  auto obstacles = nlohmann::json::array();
  for (const auto& obstacle : world.obstacles) {
    obstacles.push_back({{"min", obstacle.min}, {"max", obstacle.max}});
  }
  return {{"obstacles", obstacles}, {"start", world.start}, {"goal", world.goal}};
}

// What a plan gave: OMPL's status, what ended the plan, and the planner's exact path with
// its length and its simplification, when it found one.
struct Answer {
  ob::PlannerStatus status;
  std::string ended_by;
  std::optional<og::PathGeometric> path;
  std::optional<og::PathGeometric> simplified;
};

// One plan, built and run as ompl_bridge builds and runs it: the seed first, then the
// space, its checks, the setup, the start and the goal, and the planner.
Answer plan(const World& world, const std::string& planner, std::uint32_t seed,
            std::uint64_t iterations, std::uint32_t simplification) {
  const auto level = ompl::msg::getLogLevel();
  ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(level);

  auto space = std::make_shared<ob::RealVectorStateSpace>(world.dimension);
  space->setBounds(0.0, 1.0);
  auto si = std::make_shared<ob::SpaceInformation>(space);
  si->setMotionValidator(std::make_shared<MotionCheck>(si, world));
  si->setStateValidityChecker(std::make_shared<StateCheck>(si, world));

  og::SimpleSetup setup(si);
  setup.setStartState(state_of(space, world.start));
  auto goals = std::make_shared<ob::GoalStates>(si);
  goals->addState(state_of(space, world.goal));
  setup.setGoal(goals);
  setup.setPlanner(planner_named(planner, si));
  setup.setup();

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(time_limit_s));
  Bounds plan_bounds(iterations, deadline);
  Answer answer{setup.solve(plan_bounds.condition()), plan_bounds.ended_by(), {}, {}};
  if (answer.status != ob::PlannerStatus::EXACT_SOLUTION) {
    return answer;
  }
  const og::PathGeometric& planned = setup.getSolutionPath();
  answer.path = planned;
  if (simplification > 0) {
    og::PathGeometric simplified(planned);
    Bounds simplification_bounds(simplification, deadline);
    const bool valid =
        setup.getPathSimplifier()->simplify(simplified, simplification_bounds.condition());
    const bool better = valid && simplified.length() <= planned.length();
    answer.simplified = better ? simplified : planned;
  }
  return answer;
}

// The states of `path`, or null for a path the plan does not have.
nlohmann::json states_or_null(const std::optional<og::PathGeometric>& path,
                              std::size_t dimension) {
  return path ? states_of(*path, dimension) : nlohmann::json(nullptr);
}

// The length of `path`, or null for a path the plan does not have.
nlohmann::json length_or_null(const std::optional<og::PathGeometric>& path) {
  return path ? nlohmann::json(path->length()) : nlohmann::json(nullptr);
}

}  // namespace

nlohmann::json planning(const nlohmann::json& scenario, Inputs&) {
  ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
  const double motion_step = scenario.at("motion_step").get<double>();
  std::vector<World> worlds;
  auto echoed_worlds = nlohmann::json::array();
  for (const auto& world : scenario.at("worlds")) {
    worlds.push_back(world_of(world, motion_step));
    echoed_worlds.push_back(echo_of(worlds.back()));
  }

  auto plans = nlohmann::json::array();
  for (const auto& p : scenario.at("plans")) {
    const auto index = p.at("world").get<std::size_t>();
    const World& world = worlds.at(index);
    const auto planner = p.at("planner").get<std::string>();
    const auto seed = p.at("seed").get<std::uint32_t>();
    const auto iterations = p.at("iterations").get<std::uint64_t>();
    const auto simplification = p.at("simplification").get<std::uint32_t>();
    const Answer answer = plan(world, planner, seed, iterations, simplification);
    nlohmann::json answered = {
        {"world", index},
        {"planner", planner},
        {"seed", seed},
        {"iterations", iterations},
        {"simplification", simplification},
        {"status", answer.status.asString()},
        {"ended_by", nullptr},
        {"path", nullptr},
        {"length", nullptr},
        {"simplified", nullptr},
    };
    // PRM's path, and so its ending, depend on when its search finds a path, so a plan
    // of PRM answers its status alone, which tells whether it found one.
    if (planner != "PRM") {
      answered["ended_by"] = answer.ended_by;
      answered["path"] = states_or_null(answer.path, world.dimension);
      answered["length"] = length_or_null(answer.path);
      answered["simplified"] = states_or_null(answer.simplified, world.dimension);
    }
    plans.push_back(answered);
  }
  return {
      {"kind", "planning"},
      {"motion_step", motion_step},
      {"worlds", echoed_worlds},
      {"plans", plans},
  };
}

}  // namespace ros_reference
