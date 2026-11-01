//! The boundary with the C++ glue of `bridge.cpp`: one plan request in, one plan
//! out, and the caller's checks and constraint called back from OMPL.

use crate::callbacks::Callbacks;

#[cxx::bridge(namespace = "ompl_bridge")]
pub(crate) mod bridge {
    /// A geometric planner of OMPL, one per class.
    enum Planner {
        RrtConnect,
        Rrt,
        RrtStar,
        BiTrrt,
        Kpiece1,
        Bkpiece1,
        Lbkpiece1,
        Est,
        Sbl,
        Prm,
        PrmStar,
    }

    /// One plan, as `plan` in `bridge.cpp` builds it. States are flattened, one after
    /// another.
    struct PlanRequest {
        planner: Planner,
        low: Vec<f64>,
        high: Vec<f64>,
        start: Vec<f64>,
        goals: Vec<f64>,
        time_limit_s: f64,
        iterations: u64,
        seed: u32,
        /// 0 for no simplification.
        simplification_iterations: u32,
        /// 0 for a plan without a constraint.
        codimension: u32,
        tolerance: f64,
        projection_iterations: u32,
        /// The farthest apart two consecutive states of a constrained path lie.
        constrained_spacing: f64,
    }

    /// What OMPL's planner answered.
    enum Status {
        Exact,
        Approximate,
        NotSolved,
        InvalidStart,
        InvalidGoal,
        Other,
    }

    /// What ended the planner or the simplifier.
    enum Ending {
        Returned,
        TimeLimit,
        IterationBound,
        Failure,
    }

    struct PlanResult {
        status: Status,
        /// OMPL's name for the status.
        status_name: String,
        ended_by: Ending,
        path: Vec<f64>,
        simplified: Vec<f64>,
        /// What ended the simplifier, when the plan simplified its path.
        simplification_ended_by: Ending,
    }

    extern "Rust" {
        type Callbacks<'a>;
        fn state_valid(self: &Callbacks, state: &[f64]) -> bool;
        fn motion_valid(self: &Callbacks, from: &[f64], to: &[f64], last_valid: &mut f64) -> bool;
        fn constraint_function(self: &Callbacks, state: &[f64], out: &mut [f64]);
        fn constraint_jacobian(self: &Callbacks, state: &[f64], out: &mut [f64]);
        fn failed(self: &Callbacks) -> bool;
    }

    unsafe extern "C++" {
        include!("ompl_bridge/src/bridge.h");

        fn plan<'a>(request: &PlanRequest, callbacks: &Callbacks<'a>) -> Result<PlanResult>;
    }
}
