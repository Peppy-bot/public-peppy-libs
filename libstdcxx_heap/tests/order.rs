//! The order of `OpenSet` on drawn sequences of pushes and pops whose costs are not NaN: each
//! pop gives an entry of least cost among the entries the queue holds, so pops with no push
//! between them come out in nondecreasing cost order, and each entry pushed comes out once,
//! with the cost it went in with.

use hegel::TestCase;
use hegel::generators as gs;
use libstdcxx_heap::OpenSet;

/// Costs that drawn pushes take often, so that many entries tie: both infinities, both zeros
/// and two finite values.
const TIED_COSTS: [f32; 6] = [f32::NEG_INFINITY, -0.0, 0.0, 1.0, 2.5, f32::INFINITY];

/// The share of drawn calls that push, above one half so the queue grows over a sequence.
const PUSH_SHARE: f64 = 0.75;

/// The most calls a drawn sequence makes before the queue is drained.
const MAX_CALLS: usize = 200;

#[derive(Debug, Clone, Copy)]
enum Call {
    Push(f32),
    Pop,
}

/// A cost that is not NaN: one of [`TIED_COSTS`], or any other.
#[hegel::composite]
fn costs(tc: &TestCase) -> f32 {
    tc.draw(hegel::one_of!(
        gs::sampled_from(&TIED_COSTS),
        gs::floats::<f32>().allow_nan(false),
    ))
}

/// A push of a cost of [`costs`] at the share [`PUSH_SHARE`], a pop otherwise.
#[hegel::composite]
fn calls(tc: &TestCase) -> Call {
    if tc.draw(gs::weighted_booleans(PUSH_SHARE)) {
        Call::Push(tc.draw(costs()))
    } else {
        Call::Pop
    }
}

/// Checks that `out`, what a pop gave, is an entry of `held` of least cost, or none when `held`
/// is empty, and takes it out of `held`.
fn take_out(out: Option<(f32, u64)>, held: &mut Vec<(f32, u64)>) {
    let Some((cost, item)) = out else {
        assert!(
            held.is_empty(),
            "a pop gave none of {} entries held",
            held.len()
        );
        return;
    };
    let index = held
        .iter()
        .position(|&(_, held_item)| held_item == item)
        .unwrap_or_else(|| panic!("a pop gave item {item}, which the queue does not hold"));
    let (held_cost, _) = held.swap_remove(index);
    assert_eq!(
        cost.to_bits(),
        held_cost.to_bits(),
        "item {item} comes out with the cost it went in with"
    );
    assert!(
        held.iter().all(|&(other, _)| cost <= other),
        "item {item} of cost {cost} came out before an entry of lower cost: {held:?}"
    );
}

#[hegel::test]
fn each_pop_gives_an_entry_of_least_cost_held(tc: TestCase) {
    let calls: Vec<Call> = tc.draw(gs::vecs(calls()).max_size(MAX_CALLS));
    let mut queue = OpenSet::new();
    let mut held = Vec::new();
    for (item, call) in (0..).zip(calls) {
        match call {
            Call::Push(cost) => {
                queue.push(cost, item);
                held.push((cost, item));
            }
            Call::Pop => take_out(queue.pop(), &mut held),
        }
    }
    while !held.is_empty() {
        take_out(queue.pop(), &mut held);
    }
    assert!(
        queue.pop().is_none(),
        "the queue is empty once each entry came out"
    );
}
