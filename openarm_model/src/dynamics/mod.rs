//! Feedforward dynamics for the OpenArm control loop: gravity, Coriolis +
//! centripetal, and friction torques. All operate on the world-frame FK
//! accessors of [`crate::fk::ForwardKinematics`] (gravity points along world
//! `-z`). Gravity and Coriolis are verified against KDL reference values.

pub mod coriolis;
pub mod friction;
pub mod gravity;
