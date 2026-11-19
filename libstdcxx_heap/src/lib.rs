//! A priority queue of `(cost, item)` entries that gives them in the order of libstdc++'s
//! `std::priority_queue<std::pair<float, T>, std::vector<...>, Cmp>` of GCC 15, where
//! `Cmp(a, b)` is `a.first > b.first`. The entry of least cost comes out first, and entries
//! of equal cost come out in the order that queue gives them. Nav2's Smac 2D planner keeps
//! its open set in such a queue.
#![forbid(unsafe_code)]

/// A binary heap of `(cost, item)` entries in a vector, an entry of least cost in the first
/// slot. After the same calls, each entry holds the slot that libstdc++'s queue gives it, so
/// entries of equal cost come out in the same order. Costs compare with `>`: a NaN cost is
/// neither greater nor less than another cost.
pub struct OpenSet<T> {
    /// The entries by slot, slot 0 first.
    slots: Vec<(f32, T)>,
}

impl<T: Copy> Default for OpenSet<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Copy> OpenSet<T> {
    /// An empty queue.
    pub fn new() -> Self {
        Self { slots: Vec::new() }
    }

    /// Adds `(cost, item)`. The entry goes into a new last slot and moves up past each parent of
    /// greater cost; a parent of equal cost stops it. As `priority_queue::push`.
    pub fn push(&mut self, cost: f32, item: T) {
        let entry = (cost, item);
        self.slots.push(entry);
        self.climb(self.slots.len() - 1, entry);
    }

    /// The entry in the first slot, which the next pop gives; `None` when the queue is empty.
    /// As `priority_queue::top`.
    pub fn peek(&self) -> Option<(f32, T)> {
        self.slots.first().copied()
    }

    /// Takes out the entry in the first slot and gives it; `None` when the queue is empty. The
    /// empty first slot moves down to a leaf along the child of lesser cost, the right child when
    /// the two costs are equal, and the last entry moves up from that leaf past each parent of
    /// greater cost. As `priority_queue::top` followed by `pop`.
    pub fn pop(&mut self) -> Option<(f32, T)> {
        let last = self.slots.pop()?;
        let Some(&first) = self.slots.first() else {
            return Some(last);
        };
        let leaf = self.descend_to_leaf();
        self.climb(leaf, last);
        Some(first)
    }

    /// Moves the empty first slot down to a leaf: at each slot with two children, the entry of
    /// [`Self::lesser_child`] moves up into the empty slot; the single child of the last parent
    /// moves up with no comparison. Gives the leaf, the empty slot at the end.
    fn descend_to_leaf(&mut self) -> usize {
        let len = self.slots.len();
        let mut empty = 0;
        while right_child(empty) < len {
            let chosen = self.lesser_child(empty);
            self.slots[empty] = self.slots[chosen];
            empty = chosen;
        }
        let left = left_child(empty);
        if left < len {
            self.slots[empty] = self.slots[left];
            return left;
        }
        empty
    }

    /// Of the two children of `parent`, the left when the right is costlier than it, the right
    /// otherwise: on equal costs and when either cost is NaN.
    fn lesser_child(&self, parent: usize) -> usize {
        let left = left_child(parent);
        let right = right_child(parent);
        if is_costlier(self.slots[right].0, self.slots[left].0) {
            left
        } else {
            right
        }
    }

    /// Puts `entry` into the empty slot `empty` or an ancestor of it: while the parent of the
    /// empty slot is costlier than `entry`, the parent's entry moves down into the empty slot.
    fn climb(&mut self, mut empty: usize, entry: (f32, T)) {
        while let Some(parent) = parent_of(empty) {
            if !is_costlier(self.slots[parent].0, entry.0) {
                break;
            }
            self.slots[empty] = self.slots[parent];
            empty = parent;
        }
        self.slots[empty] = entry;
    }
}

/// Whether `cost` is greater than `other` under `>` on `f32`: false when either is NaN, and
/// false both ways for `-0.0` and `0.0`. Each decision of a push or a pop is this test.
fn is_costlier(cost: f32, other: f32) -> bool {
    cost > other
}

fn left_child(slot: usize) -> usize {
    2 * slot + 1
}

fn right_child(slot: usize) -> usize {
    2 * slot + 2
}

/// The parent of `slot`; slot 0 has none.
fn parent_of(slot: usize) -> Option<usize> {
    slot.checked_sub(1).map(|above| above / 2)
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use serde_json::Value;

    use super::*;

    /// Vectors that `std::priority_queue` of GCC 15 gave, one step per line, a subset of the
    /// full file: the empty queue, 2 to 6 entries of one cost, a lower cost after equal ones,
    /// children of equal cost under a lower parent, pushes between pops, signed zeros,
    /// infinities, and NaN costs pushed last, pushed first and met in a descent.
    const COMMITTED_VECTORS: &str = include_str!("../tests/fixtures/vectors.json");

    /// The environment variable that names the full vector file.
    const FULL_VECTORS_VARIABLE: &str = "LIBSTDCXX_HEAP_VECTORS";

    /// Costs whose bits hold more than their value: quiet NaNs of both signs with payloads of
    /// their own, and both zeros.
    const COST_BITS: [u32; 4] = [0x7fc0_1234, 0xffc0_4321, 0x8000_0000, 0x0000_0000];

    /// An entry as a vector writes it, its item a `u64` as the A* search's cell index is.
    type Entry = (f32, u64);

    /// A call of a vector's step, with what the C++ queue gave for it.
    enum Call {
        Push(Entry),
        /// The entry in the first slot, which the C++ program read with `top`; `None` when the
        /// queue is empty.
        Peek(Option<Entry>),
        Pop(Option<Entry>),
    }

    /// A step: its call, and the slots after it, slot 0 first.
    struct Step {
        call: Call,
        layout: Vec<Entry>,
    }

    struct Vector {
        name: String,
        steps: Vec<Step>,
    }

    fn parse_vectors(text: &str) -> Vec<Vector> {
        let file: Value = serde_json::from_str(text).expect("the vector file is JSON");
        file["vectors"]
            .as_array()
            .expect("the vector file holds an array `vectors`")
            .iter()
            .map(parse_vector)
            .collect()
    }

    fn parse_vector(value: &Value) -> Vector {
        let name = value["name"].as_str().expect("a vector has a name");
        let steps = value["ops"]
            .as_array()
            .unwrap_or_else(|| panic!("{name} holds an array `ops`"))
            .iter()
            .map(parse_step)
            .collect();
        Vector {
            name: name.to_owned(),
            steps,
        }
    }

    fn parse_step(value: &Value) -> Step {
        let call = match value["op"].as_str() {
            Some("push") => Call::Push((parse_cost(&value["cost"]), parse_item(&value["item"]))),
            Some("peek") => Call::Peek(parse_out(&value["out"])),
            Some("pop") => Call::Pop(parse_out(&value["out"])),
            other => panic!("a step's op is push, peek or pop, not {other:?}"),
        };
        let layout = value["layout"]
            .as_array()
            .unwrap_or_else(|| panic!("a step's layout is an array, not {}", value["layout"]))
            .iter()
            .map(parse_entry)
            .collect();
        Step { call, layout }
    }

    /// A cost: a JSON number, or one of the strings `"NaN"`, `"inf"` and `"-inf"`.
    fn parse_cost(value: &Value) -> f32 {
        match value {
            Value::Number(number) => number.as_f64().expect("a JSON number is an f64") as f32,
            Value::String(text) if text == "NaN" => f32::NAN,
            Value::String(text) if text == "inf" => f32::INFINITY,
            Value::String(text) if text == "-inf" => f32::NEG_INFINITY,
            other => panic!("a cost is a number, \"NaN\", \"inf\" or \"-inf\", not {other}"),
        }
    }

    fn parse_item(value: &Value) -> u64 {
        value
            .as_u64()
            .unwrap_or_else(|| panic!("an item is a whole number, not {value}"))
    }

    /// An entry written `[cost, item]`.
    fn parse_entry(value: &Value) -> Entry {
        match value.as_array().map(Vec::as_slice) {
            Some([cost, item]) => (parse_cost(cost), parse_item(item)),
            _ => panic!("an entry is [cost, item], not {value}"),
        }
    }

    /// What a peek or a pop gave: an entry, or `null` for none.
    fn parse_out(value: &Value) -> Option<Entry> {
        (!value.is_null()).then(|| parse_entry(value))
    }

    /// Whether two costs are the same `f32` with the same sign, or both NaN.
    fn same_cost(a: f32, b: f32) -> bool {
        a.to_bits() == b.to_bits() || (a.is_nan() && b.is_nan())
    }

    fn same_entry(a: Entry, b: Entry) -> bool {
        same_cost(a.0, b.0) && a.1 == b.1
    }

    fn same_out(a: Option<Entry>, b: Option<Entry>) -> bool {
        match (a, b) {
            (Some(a), Some(b)) => same_entry(a, b),
            (None, None) => true,
            _ => false,
        }
    }

    fn same_layout(a: &[Entry], b: &[Entry]) -> bool {
        a.len() == b.len() && a.iter().zip(b).all(|(&a, &b)| same_entry(a, b))
    }

    /// Makes `step`'s call on `queue`; an error names the output or the slots that differ from
    /// what the C++ queue gave and left.
    fn make_step(queue: &mut OpenSet<u64>, step: &Step) -> Result<(), String> {
        let output = match step.call {
            Call::Push((cost, item)) => {
                queue.push(cost, item);
                None
            }
            Call::Peek(expected) => Some((queue.peek(), expected)),
            Call::Pop(expected) => Some((queue.pop(), expected)),
        };
        if let Some((gave, expected)) = output
            && !same_out(gave, expected)
        {
            return Err(format!("gave {gave:?}, the C++ queue gave {expected:?}"));
        }
        if !same_layout(&queue.slots, &step.layout) {
            return Err(format!(
                "left the slots {:?}, the C++ queue left {:?}",
                queue.slots, step.layout
            ));
        }
        Ok(())
    }

    /// Makes each step of `vector` on an empty queue; an error names the first step whose
    /// output or slots differ from the vector's.
    fn replay(vector: &Vector) -> Result<(), String> {
        let mut queue = OpenSet::new();
        vector
            .steps
            .iter()
            .enumerate()
            .try_for_each(|(index, step)| {
                make_step(&mut queue, step)
                    .map_err(|mismatch| format!("{} step {index}: {mismatch}", vector.name))
            })
    }

    /// Replays every vector of `text`, a vector file, and fails with the first mismatch of each
    /// vector that differs.
    fn assert_every_vector_replays(text: &str) {
        let vectors = parse_vectors(text);
        assert!(!vectors.is_empty(), "the vector file holds vectors");
        let mismatches: Vec<String> = vectors
            .iter()
            .filter_map(|vector| replay(vector).err())
            .collect();
        assert!(
            mismatches.is_empty(),
            "{} of {} vectors differ:\n{}",
            mismatches.len(),
            vectors.len(),
            mismatches.join("\n")
        );
    }

    #[test]
    fn the_committed_vectors_replay_step_for_step() {
        assert_every_vector_replays(COMMITTED_VECTORS);
    }

    /// Every vector of the full file that `tests/fixtures/gen_vectors.cpp` writes, read from the
    /// path that the environment variable `LIBSTDCXX_HEAP_VECTORS` names. The program drives
    /// GCC 15's `std::priority_queue` through 31 vectors and prints the file; built and run in
    /// a directory `$dir` outside the repository, from the crate's directory:
    /// `g++ -std=c++20 -O2 -Wall -Wextra -o "$dir/gen_vectors" tests/fixtures/gen_vectors.cpp`,
    /// then `"$dir/gen_vectors" > "$dir/vectors.json"`, and the test runs with
    /// `LIBSTDCXX_HEAP_VECTORS="$dir/vectors.json" cargo test -- --ignored`. The full file stays
    /// out of the repository, so this test is ignored. `tests/fixtures/vectors.json`, which
    /// [`COMMITTED_VECTORS`] reads, is the full file with 14 of its vectors kept, each as the
    /// program wrote it.
    #[test]
    #[ignore = "reads the full vector file that LIBSTDCXX_HEAP_VECTORS names"]
    fn every_vector_of_the_full_file_replays_step_for_step() {
        let path = std::env::var_os(FULL_VECTORS_VARIABLE)
            .unwrap_or_else(|| panic!("{FULL_VECTORS_VARIABLE} names the full vector file"));
        let path = Path::new(&path);
        let text = std::fs::read_to_string(path)
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        assert_every_vector_replays(&text);
    }

    #[test]
    fn every_cost_comes_out_bit_for_bit() {
        let pushed: Vec<(u32, u64)> = COST_BITS.into_iter().zip(0..).collect();
        let mut queue = OpenSet::new();
        for &(bits, item) in &pushed {
            queue.push(f32::from_bits(bits), item);
        }
        let mut popped: Vec<(u32, u64)> = std::iter::from_fn(|| queue.pop())
            .map(|(cost, item)| (cost.to_bits(), item))
            .collect();
        popped.sort_unstable_by_key(|&(_, item)| item);
        assert_eq!(popped, pushed);
    }

    #[test]
    fn default_is_an_empty_queue() {
        assert!(OpenSet::<u64>::default().pop().is_none());
    }
}
