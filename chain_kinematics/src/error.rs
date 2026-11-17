//! What can go wrong building a chain. Each variant names the thing the caller
//! got wrong, so a message is actionable without reading this crate.

/// A chain could not be built from the URDF and spec given.
#[derive(Debug, thiserror::Error)]
pub enum ChainError {
    #[error("URDF has no link named '{0}'")]
    NoSuchLink(String),
    #[error("tip link '{tip}' is not below base link '{base}'")]
    TipNotBelowBase { tip: String, base: String },
    #[error("joint '{0}' is not on the path from the base to the tip")]
    JointNotOnPath(String),
    #[error("joint '{0}' is fixed, so it cannot be actuated")]
    JointDoesNotMove(String),
    #[error("joint '{0}' is named twice; each entry of `q` drives one joint")]
    DuplicateJoint(String),
    #[error(
        "joint '{follower}' mimics '{leader}', and this chain actuates one of them; \
         the coupled joint would be left behind"
    )]
    MimicCoupling { follower: String, leader: String },
    #[error(
        "the actuated joint '{joint}' has no range: it is continuous, or its <limit> gives \
         its lower and upper limits equal, as an absent <limit> does"
    )]
    NoRange { joint: String },
    #[error("'{tool}' is not a link fixed below the tip '{tip}'")]
    ToolNotFixedBelowTip { tool: String, tip: String },
    #[error("chain has {found} actuated joints, expected {expected}")]
    JointCount { expected: usize, found: usize },
    #[error(transparent)]
    Tree(#[from] TreeError),
}

/// A URDF whose links and joints are not one tree of joints that each take one
/// position ([`Tree::from_robot`](crate::Tree::from_robot)), in the words of the URDF.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum TreeError {
    #[error("the link {0:?} is given twice")]
    LinkTwice(String),
    #[error("the joint {0:?} is given twice")]
    JointTwice(String),
    #[error("the joint {joint:?} names the link {link:?}, and the URDF has no such link")]
    NoSuchLink { joint: String, link: String },
    #[error(
        "the link {link:?} hangs on the joints {first:?} and {second:?}; a link hangs on one joint"
    )]
    LinkOnTwoJoints {
        link: String,
        first: String,
        second: String,
    },
    #[error(
        "the joints {} join their links in a loop; every link hangs below the root link",
        .joints.join(", ")
    )]
    JointLoop { joints: Vec<String> },
    #[error(
        "{}; a URDF has one root link, and every other link hangs on a joint",
        root_links(.roots)
    )]
    RootLinks { roots: Vec<String> },
    #[error(
        "the joint {joint:?} is {kind}, which takes more than one position; a joint is \
         revolute, continuous, prismatic or fixed"
    )]
    MultiPositionJoint { joint: String, kind: String },
    #[error(
        "the joint {joint:?} moves about or along the axis {axis:?}, which gives no direction; \
         a joint that moves has a finite axis that is not zero"
    )]
    JointAxis { joint: String, axis: [f64; 3] },
    #[error(
        "the joint {joint:?} moves between the limits {lower} and {upper}; a revolute or a \
         prismatic joint has finite limits, the lower one at or below the upper one"
    )]
    JointLimits {
        joint: String,
        lower: f64,
        upper: f64,
    },
    #[error("the joint {joint:?} has {error}")]
    JointMimic { joint: String, error: MimicError },
}

/// A mimic that [`Mimic::new`](crate::Mimic::new) refuses: its multiplier or its offset
/// is not finite.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[error(
    "the mimic of {joint:?} at the multiplier {multiplier} and the offset {offset}; a \
     mimic's multiplier and offset are finite"
)]
pub struct MimicError {
    pub joint: String,
    pub multiplier: f64,
    pub offset: f64,
}

/// The links that hang on no joint, as a refusal writes them.
fn root_links(roots: &[String]) -> String {
    match roots {
        [] => "the URDF has no link".to_owned(),
        _ => format!("the links {} hang on no joint", roots.join(", ")),
    }
}
