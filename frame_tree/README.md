# frame_tree

tf2's buffer in Rust. A node that reads the transforms contract takes each message of
`transforms` and of `static_transforms` into a `FrameTree`, and asks it for the place of
any frame in any other, at a time or at the latest time common to the chain. The crate
also holds tf2's `Transform`, a rotation matrix and an origin that places a point and
composes on tf2's bits, and `arithmetic`, the rotation arithmetic of tf2 and KDL operation
for operation and the angles library's `normalize_angle`, which the libraries' geometry runs
on. Pure Rust; no hardware or messaging dependencies.

## Quick start

```rust
use std::time::Duration;
use frame_tree::{
    FrameName, FrameTransform, FrameTree, LookupTime, MovingMessage, Publisher, Stamp,
    StaticMessage,
};

// The window is the caller's, above 0 s: how long each moving transform keeps its stamps.
let mut tree = FrameTree::new(Duration::from_secs(10))?;

// Who sent a message: the core node and the instance id of its producer.
let publisher = Publisher { core_node, instance_id };

// The transforms of a message of static_transforms: its publisher's whole static set,
// each a TransformRecord { frame, child_frame, position, orientation }.
let refused = tree.take_static(&publisher, &StaticMessage { transforms })?;

// A message of transforms: the transforms its publisher measured at its timestamp, and
// when they last started over, each time a SystemTime as the message carries it.
let refused = tree.take_moving(&publisher, &MovingMessage {
    timestamp, transforms, start_timestamp,
})?;

// When the consumer's clock goes back: every moving stamp goes, the static transforms stay.
tree.clear_moving();

// Where the lidar stood in odom when it took a scan.
let found = tree.lookup(&FrameName::parse("odom")?, &FrameName::parse("lidar_link")?,
                        LookupTime::At(Stamp::from_time(scanned)?))?;

// A publisher's side: its transforms as the fields of its message.
let transforms = vec![FrameTransform::parse("odom", "base_footprint", position, orientation)?];
let message = MovingMessage::from_transforms(measured, &transforms, started)?;
```

A message holds its transforms as records, `TransformRecord`: each a parent frame
(`frame`), a `child_frame`, a `position` and an `orientation` ([x, y, z, w]). A message of
`transforms` also carries `timestamp`, when its publisher measured all its transforms, and
`start_timestamp`, when they last started over. A message that names a child frame twice,
or whose `timestamp` or `start_timestamp` lies before the Unix epoch or more than
`u64::MAX` nanoseconds after it, is refused whole (`MessageError`) and changes nothing;
`from_transforms` refuses a publisher's message on the same times. A transform with an
empty frame, its child frame for its parent, a number that is not finite, or a quaternion
whose squared norm lies 10e-3 or more from 1 is refused alone, and so is a moving one
whose stamp is older than its child frame's newest by more than the window; `take_moving`
and `take_static` give each refused transform with its index and its reason
(`TransformError`), and the message's other transforms are kept, each with its quaternion
normalized.

A `Stamp` holds a time from the Unix epoch to `u64::MAX` nanoseconds after it;
`Stamp::from_time` refuses any other with `TimeError`.

A start from a publisher that differs from the one the tree holds of a child frame from
that publisher drops every stamp of the child frame held from that publisher before the
new stamp goes in, so no lookup interpolates across a start. Two starts are compared for
equality alone, and the first message from a publisher has no start to compare.

## The rules of tf2 it follows

Each rule is tf2's, geometry2 0.45.10, the release ROS 2 Lyrical ships, by file and line
under `tf2/`.

| Rule | tf2 |
|---|---|
| A transform is refused when its child frame is its parent, a frame is empty, a coordinate is NaN, or its quaternion's squared norm differs from 1 by 10e-3 or more | `src/buffer_core.cpp:66`, `:210-266` |
| A moving transform keeps its stamps newest first; on an arriving stamp the stamps older than the newest held by more than the window go, then a stamp older than that is refused | `src/cache.cpp:246-264`, `:336-343` |
| A time on the newest or the oldest stamp takes that sample; a time between two samples interpolates them, linearly in position and by tf2's slerp along the shorter arc in orientation; where the two have different parents, the older sample holds | `src/cache.cpp:114-230`, `include/tf2/LinearMath/Quaternion.hpp:340-364`, `include/tf2/LinearMath/Vector3.hpp:222-230` |
| A moving transform with one sample has a value at its stamp alone; past the newest or before the oldest it has none | `src/cache.cpp:134-165` |
| A moving transform with no stamp has no value at any time, and gives its child frame no parent at the latest time | `src/cache.cpp:122-126`, `:303-307`, `src/buffer_core.cpp:1147-1150` |
| A static transform holds at every time | `src/static_cache.cpp:37-54` |
| A transform sent static then moving holds as moving, and the other way round | `src/buffer_core.cpp:276-285` |
| A frame in itself is the identity, whether the tree knows the frame or not; at the latest time its time is its transform's newest stamp | `src/buffer_core.cpp:830-846` |
| A lookup of a frame the tree does not know fails, the target checked first | `src/buffer_core.cpp:148-153`, `:849-850` |
| A lookup reads the source's chain of parents up to its root, or up to the first transform it cannot read, then the target's chain up to where the source's stopped, and composes them; where the target is an ancestor of the source, or the source of the target, it stops there | `src/buffer_core.cpp:330-522` |
| Where both chains stop at a transform with no value, the error is the transform whose newest sample is newer, the source's on a tie | `src/buffer_core.cpp:426-445` |
| Where the chains do not meet, the frames are in two trees; a missing value comes before that | `src/buffer_core.cpp:481-493`, `:1047-1058` |
| The composition is tf2's `TransformAccum`, quaternion and vector arithmetic operation for operation | `src/buffer_core.cpp:524-596`, `include/tf2/LinearMath/Quaternion.hpp:144-151`, `:427-443`, `:502-507` |
| A lookup of the source at one time in the target at another (`lookup_through`) checks that the tree knows its target, its source and its fixed frame, in that order, then goes through the fixed frame: the source in the fixed frame at the source's time and the fixed frame in the target at the target's time, each made a `Transform` by `setRotation`, then their `Transform` product, its rotation read back by `getRotation`; its time is the target side's | `src/buffer_core.cpp:750-781`, `:794-805`, `:880-898`, `:132-155`, `include/tf2/LinearMath/Transform.hpp:124-128`, `:169-172`, `:252-257` |
| The root of a frame's chain at a time (`root_at`) is the frame where the walk of a lookup's source from it ends; a transform with no value at the time fails it as a lookup fails, and tf2's walk stops at that transform | `src/buffer_core.cpp:352-397` |
| The latest time of a lookup is the earliest of the newest stamps of the moving transforms on both chains up to their nearest common ancestor; a chain of static transforms has none, and its lookup holds at every time | `src/buffer_core.cpp:1112-1261` |

`tests/ros_reference_transform_lookup.rs` holds the crate to these rules against tf2
itself, on the transform lookup scenarios of the ROS reference harness
(`ros_reference/`): every transform taken or refused as tf2 takes it, every lookup and every
lookup through a fixed frame refused with the kind of error tf2 throws, and every other one
at tf2's time, within 1e-9 m and 1e-9 rad of tf2's transform.

## Where it departs from tf2

- **Every transform records its publisher, and transforms of one child frame from two
  publishers break the tree there while both are held.** A static transform is held until
  its publisher's next static set leaves it out; a moving transform, while its newest
  stamp lies within the window of the newest stamp of the child frame from either
  publisher, and it goes at the next stamp past that, as a stamp past the window goes. A
  lookup that reads such a frame fails, naming it and both publishers, and the tree
  recovers once one goes. tf2 takes the samples of both and names the last publisher in
  its debugging output (`src/buffer_core.cpp:293`); each transform of the contract has one
  publisher.
- **Transforms that form a cycle break the tree.** A lookup whose walk comes back to a
  frame it passed fails, naming every transform of the cycle. tf2 walks on until its depth
  limit of 1000 frames and reports a loop (`include/tf2/buffer_core.hpp:99`,
  `src/buffer_core.cpp:398-407`).
- **A walk stops at a broken transform as at a transform with no value.** A lookup fails
  on a broken transform only where it needs it: the frames below it that meet below it
  look each other up. Where a lookup meets a broken transform and another failure, it
  names the one the walks meet first, the source walk's before the target walk's.
- **A transform starts over.** tf2 has no counterpart: a new start of a child frame's
  transform from a publisher drops that publisher's stamps of the child frame.
- **An unknown frame's error names every frame the tree holds**, sorted, so it says what
  to look up. tf2's says the frame does not exist (`src/buffer_core.cpp:148-153`).
- **A stamp sent again replaces the held one.** tf2 keeps both samples of one stamp
  (`src/cache.cpp:274-288`).
- **A publisher's static set replaces its last one.** A message of `static_transforms`
  carries its publisher's whole static set, so a transform the set omits is dropped. tf2
  drops no static transform, and its static broadcaster sends its whole set each time
  (`tf2_ros/src/static_transform_broadcaster.cpp:52-71`). An invalid transform of a set is
  refused, so the transform the publisher held of its child frame goes too.
- **A clear drops the moving stamps and keeps the static transforms.** `clear_moving`
  drops every moving stamp, as a consumer does when its clock goes back, which the
  consumer detects, as tf2_ros's buffer detects a jump back in time at its clock
  (`tf2_ros/src/buffer.cpp:70-80`, `:106-116`). tf2's clear empties its static caches too
  (`src/buffer_core.cpp:171-183`, `src/static_cache.cpp:63`); a static transform holds at
  every time. A cleared transform keeps its frames known.
- **A window of 0 s is refused.** `FrameTree::new` takes the window from its caller and
  refuses 0 s with `ZeroWindow`, as the transforms contract asks for a window above 0 s.
  tf2's buffer takes any window and defaults to 10 s (`include/tf2/buffer_core.hpp:73`,
  `src/buffer_core.cpp:158-167`).
- **A frame name is the URDF's, byte for byte.** A name that starts with `/` names a frame
  of its own. tf2 strips the `/` from a transform's frames (`src/buffer_core.cpp:212-213`)
  and refuses it in a lookup (`:142-146`).
- **Every number is finite.** tf2 refuses a NaN and stores an infinite coordinate
  (`src/buffer_core.cpp:238-248`); a position of the contract is in meters.
- **A quaternion is normalized.** tf2 keeps a quaternion within its tolerance as given;
  the tree multiplies it by the reciprocal of its length, as tf2's `normalize` does
  (`include/tf2/LinearMath/Quaternion.hpp:181-184`, `:207-219`).
- **A stamp of zero is a time like any other.** tf2 reads the time zero as the latest
  time (`src/cache.cpp:128-132`) and leaves a stamp of zero out of the latest common time
  (`src/buffer_core.cpp:1152-1154`); a lookup here names the latest time as
  `LookupTime::Latest`.
- **A lookup of a frame in itself at the latest time has no time where the frame's
  transform has two publishers.**

## Tests

```sh
cargo test
```

- `src/` unit tests: the arithmetic, `Transform`, the time cache, the parsing of a message,
  a frame name and a stamp.
- `tests/tree.rs`: each rule on a small tree, and each departure.
- `tests/properties.rs`: Hegel properties on drawn trees: a lookup and its reverse undo
  each other, lookups compose along a chain, a static chain holds at every time, a moving
  transform blends its two samples between their stamps and has no value outside them,
  and a lookup never blends a transform's samples across a start.
- `tests/ros_reference_transform_lookup.rs`: the parity with tf2.
