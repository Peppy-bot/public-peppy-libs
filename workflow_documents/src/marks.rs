//! The marks of the workflow side: names that a contract gives a field
//! (`$marks`) or a member (`marks`), which the checker of the workflow
//! director reads. The mark list of contracts-hub names each with the owner
//! `workflow`.

/// A field that holds a position or a pose in the robot frame: a workflow
/// gives it only a `$ref` to a field with this mark of a call or an
/// observation of the same role.
pub const ROBOT_FRAME: &str = "robot_frame";

/// A field that holds an item id, valid only on the node instance that gives
/// it: a workflow gives it only a `$ref` to a field with this mark of a call
/// or an observation on the same instance.
pub const ITEM_ID: &str = "item_id";

/// A member that reaches for an item: a lane that calls it has a reach
/// assumption whose check reads an observation of its role.
pub const REACHES_FOR_ITEM: &str = "reaches_for_item";

/// A string field that receives the name of a location the node knows
/// through locations:v1: a workflow gives it only a `{ $location }` of a
/// declared location, and the instance that serves the member implements
/// locations:v1.
pub const LOCATION: &str = "location";
