# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the transform lookup scenarios of the ROS reference harness.

transform_lookup_drawn_tree draws a tree of frames, each frame's parent drawn among the
frames before it, each transform static or moving. Each tick sends a sample of most
moving transforms at a stamp of its own, its orientation drawn uniformly or the
orientation before it; some ticks send a sample older than the newest; one tick moves a transform to
another parent. The stamps of each child frame are distinct: frame_tree replaces a held
stamp sent again, where tf2 keeps both. After each tick, lookups between drawn frames: at
the latest time, at a drawn time around the samples held, some of them past the window,
and at a stamp sent.

transform_lookup_rules sends a few steps for each rule of tf2's buffer: a frame in itself,
an unknown frame, an empty frame name, two trees, a cycle, a transform with one sample,
a transform's oldest and newest samples, a sample older than the window, a transform sent
static then moving and static again, a change of parent between two samples, and invalid
transforms.

Run it from any directory:

    uv run ros_reference/tools/draw_transform_lookup.py
    uv run ros_reference/tools/draw_transform_lookup.py --drawn 40

The second draws a large set for a local run, which the README describes.
"""

from drawing import generator, options, unit_quaternion, write

DRAWN_TICKS = 8
FRAMES = 8
# The base stamp and the spacing of the ticks, in nanoseconds.
START_NS = 1_000_000_000
TICK_NS = 100_000_000
WINDOW_NS = 500_000_000
LOOKUPS_PER_TICK = 3


def frame(index):
    return f"frame_{index}"


def transform(step, parent, child, position, orientation, **stamp):
    return {"step": step, **stamp, "parent": parent, "child": child,
            "position": position, "orientation": orientation}


def lookup(target, source, time):
    return {"step": "lookup", "target": target, "source": source, "time": time}


def position(rng):
    return [rng.uniform(-1.0, 1.0) for _ in range(3)]


def drawn_tree(rng, ticks):
    parents = {index: rng.randrange(index) for index in range(1, FRAMES)}
    moving = sorted(index for index in parents if rng.random() < 0.6)
    steps = [
        transform("static", frame(parent), frame(child), position(rng), unit_quaternion(rng))
        for child, parent in parents.items()
        if child not in moving
    ]
    orientations = {child: unit_quaternion(rng) for child in moving}
    sent = []
    # The stamps sent of each child frame.
    stamps = {child: set() for child in moving}

    def send(child, stamp, orientation):
        while stamp in stamps[child]:
            stamp += 1
        stamps[child].add(stamp)
        sent.append(stamp)
        steps.append(transform("moving", frame(parents[child]), frame(child),
                               position(rng), orientation, stamp=stamp))

    reparent_tick = rng.randrange(1, ticks)
    for tick in range(ticks):
        tick_ns = START_NS + tick * TICK_NS + rng.randrange(-20_000_000, 20_000_000)
        if tick == reparent_tick:
            child = rng.choice(moving)
            parents[child] = rng.randrange(child)
        for child in rng.sample(moving, len(moving)):
            if rng.random() < 0.2:
                continue
            if rng.random() < 0.15:
                orientation = orientations[child]
            else:
                orientation = unit_quaternion(rng)
            orientations[child] = orientation
            send(child, tick_ns + rng.randrange(0, 10_000_000), orientation)
        if tick > 0 and rng.random() < 0.3:
            child = rng.choice(moving)
            send(child, tick_ns - rng.randrange(TICK_NS, 2 * TICK_NS), unit_quaternion(rng))
        for _ in range(LOOKUPS_PER_TICK):
            target, source = rng.sample(range(FRAMES), 2)
            time = rng.choice([
                None,
                tick_ns + rng.randrange(-WINDOW_NS - TICK_NS, TICK_NS // 2),
                rng.choice(sent) if sent else None,
            ])
            steps.append(lookup(frame(target), frame(source), time))
    return {"kind": "transform_lookup", "cache_window_ns": WINDOW_NS, "steps": steps}


def rules(rng):
    q = lambda: unit_quaternion(rng)
    t = lambda: position(rng)
    at = lambda tick: START_NS + tick * TICK_NS
    steps = [
        # Two trees: root -> a -> b static, other -> c moving at one stamp.
        transform("static", "root", "a", t(), q()),
        transform("static", "a", "b", t(), q()),
        transform("moving", "other", "c", t(), q(), stamp=at(1)),
        lookup("b", "c", at(1)),
        lookup("b", "c", None),
        lookup("root", "b", None),
        lookup("b", "root", at(5)),
        # A frame in itself, known or not.
        lookup("b", "b", None),
        lookup("c", "c", None),
        lookup("nowhere", "nowhere", at(1)),
        # An unknown frame, as the target and as the source, and an empty name.
        lookup("nowhere", "b", None),
        lookup("b", "nowhere", at(1)),
        lookup("", "b", None),
        # One sample holds at its stamp alone.
        lookup("other", "c", at(1)),
        lookup("other", "c", at(1) - 1),
        lookup("other", "c", at(1) + 1),
        lookup("c", "other", None),
        # Two samples: before the oldest, after the newest, between them.
        transform("moving", "other", "c", t(), q(), stamp=at(3)),
        lookup("other", "c", at(0)),
        lookup("c", "other", at(4)),
        lookup("other", "c", at(2)),
        # A sample older than the newest by more than the window is refused; the window
        # prunes the oldest samples at the next sample.
        transform("moving", "other", "c", t(), q(), stamp=at(3) - WINDOW_NS - 1),
        transform("moving", "other", "c", t(), q(), stamp=at(3) - WINDOW_NS),
        transform("moving", "other", "c", t(), q(), stamp=at(9)),
        transform("moving", "other", "c", t(), q(), stamp=at(10)),
        lookup("other", "c", at(3)),
        lookup("other", "c", at(5)),
        # A change of parent between two samples: the older sample holds between them.
        transform("moving", "root", "d", t(), q(), stamp=at(1)),
        transform("moving", "a", "d", t(), q(), stamp=at(3)),
        lookup("root", "d", at(2)),
        lookup("a", "d", at(2)),
        lookup("a", "d", at(3)),
        lookup("b", "d", None),
        # A transform sent static, then moving, then static again.
        transform("static", "b", "e", t(), q()),
        lookup("b", "e", at(7)),
        transform("moving", "b", "e", t(), q(), stamp=at(6)),
        lookup("b", "e", at(7)),
        lookup("b", "e", at(6)),
        transform("static", "b", "e", t(), q()),
        lookup("b", "e", at(7)),
        # A cycle: f -> g -> h -> f, with a leaf below it.
        transform("static", "g", "f", t(), q()),
        transform("static", "h", "g", t(), q()),
        transform("static", "f", "h", t(), q()),
        transform("static", "f", "leaf", t(), q()),
        lookup("g", "leaf", None),
        lookup("root", "leaf", None),
        lookup("root", "leaf", at(1)),
        lookup("leaf", "root", at(1)),
        # Invalid transforms: a frame its own parent, an empty frame, a quaternion off unit.
        transform("moving", "root", "root", t(), q(), stamp=at(2)),
        transform("moving", "root", "", t(), q(), stamp=at(2)),
        transform("moving", "", "x", t(), q(), stamp=at(2)),
        transform("moving", "root", "x", t(), [0.0, 0.0, 0.0, 1.01], stamp=at(2)),
        lookup("root", "x", at(2)),
    ]
    return {"kind": "transform_lookup", "cache_window_ns": WINDOW_NS, "steps": steps}


def main():
    ticks = options(__doc__, DRAWN_TICKS, "ticks of the drawn tree").parse_args().drawn
    name = "transform_lookup_drawn_tree"
    write(name, drawn_tree(generator(name), ticks))
    name = "transform_lookup_rules"
    write(name, rules(generator(name)))


if __name__ == "__main__":
    main()
