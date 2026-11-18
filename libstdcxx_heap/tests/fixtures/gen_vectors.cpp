// Drives std::priority_queue<std::pair<float, int>, std::vector<...>, GreaterCost>,
// GreaterCost being `a.first > b.first`, through sequences of pushes, peeks and pops,
// and prints each step with what the queue gave as JSON on standard output.
//
// An item is the number of pushes before it in its sequence: 0, 1, 2 and so on.
// A peek or pop on an empty queue is answered `null` here, without calling the queue.
//
// Build and run with the binary and the full vector file in a directory `$dir` outside the
// repository, whose tests/fixtures/vectors.json holds 14 of the vectors as this program
// writes them:
//   g++ -std=c++20 -O2 -Wall -Wextra -o "$dir/gen_vectors" gen_vectors.cpp
//   "$dir/gen_vectors" > "$dir/vectors.json"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace {

using Entry = std::pair<float, int>;

struct GreaterCost {
  bool operator()(const Entry& a, const Entry& b) const { return a.first > b.first; }
};

// The queue, with its underlying vector readable for the layout of each step.
struct LayoutQueue : std::priority_queue<Entry, std::vector<Entry>, GreaterCost> {
  const std::vector<Entry>& layout() const { return c; }
};

enum class Kind { Push, Peek, Pop };

struct Op {
  Kind kind;
  float cost;
};

struct Sequence {
  std::string name;
  std::string about;
  std::vector<Op> ops;
};

const float kNaN = std::numeric_limits<float>::quiet_NaN();
const float kInf = std::numeric_limits<float>::infinity();

Op push(float cost) { return {Kind::Push, cost}; }
Op peek() { return {Kind::Peek, 0.0f}; }
Op pop() { return {Kind::Pop, 0.0f}; }

// A finite cost as a JSON number with a decimal point, so -0 keeps its sign through a
// parser; NaN and the infinities as the strings "NaN", "inf" and "-inf".
std::string cost_json(float cost) {
  if (std::isnan(cost)) return "\"NaN\"";
  if (std::isinf(cost)) return cost > 0 ? "\"inf\"" : "\"-inf\"";
  char text[64];
  std::snprintf(text, sizeof text, "%.9g", static_cast<double>(cost));
  std::string number = text;
  if (number.find_first_of(".e") == std::string::npos) number += ".0";
  return number;
}

std::string entry_json(const Entry& entry) {
  return "[" + cost_json(entry.first) + ", " + std::to_string(entry.second) + "]";
}

std::string layout_json(const LayoutQueue& queue) {
  std::string text = "[";
  for (std::size_t slot = 0; slot < queue.layout().size(); ++slot) {
    if (slot > 0) text += ", ";
    text += entry_json(queue.layout()[slot]);
  }
  return text + "]";
}

std::string json_string(const std::string& text) {
  std::string quoted = "\"";
  for (char c : text) {
    if (c == '"' || c == '\\') quoted += '\\';
    quoted += c;
  }
  return quoted + "\"";
}

// One line per step: the step, what it gave, and the queue's vector after it.
std::string run_json(const Sequence& sequence) {
  LayoutQueue queue;
  int pushed = 0;
  std::string text = "    {\"name\": " + json_string(sequence.name) +
                     ",\n     \"about\": " + json_string(sequence.about) + ",\n     \"ops\": [\n";
  for (std::size_t index = 0; index < sequence.ops.size(); ++index) {
    const Op& op = sequence.ops[index];
    std::string line;
    switch (op.kind) {
      case Kind::Push:
        queue.emplace(op.cost, pushed);
        line = "{\"op\": \"push\", \"cost\": " + cost_json(op.cost) +
               ", \"item\": " + std::to_string(pushed) + ", ";
        ++pushed;
        break;
      case Kind::Peek:
        line = "{\"op\": \"peek\", \"out\": " +
               (queue.empty() ? std::string("null") : entry_json(queue.top())) + ", ";
        break;
      case Kind::Pop:
        if (queue.empty()) {
          line = "{\"op\": \"pop\", \"out\": null, ";
        } else {
          line = "{\"op\": \"pop\", \"out\": " + entry_json(queue.top()) + ", ";
          queue.pop();
        }
        break;
    }
    line += "\"layout\": " + layout_json(queue) + "}";
    text += "       " + line + (index + 1 < sequence.ops.size() ? ",\n" : "\n");
  }
  return text + "     ]}";
}

std::vector<Op> pushes(const std::vector<float>& costs) {
  std::vector<Op> ops;
  for (float cost : costs) ops.push_back(push(cost));
  return ops;
}

std::vector<Op> pops(int count) { return std::vector<Op>(count, pop()); }

std::vector<Op> concat(std::vector<Op> first, const std::vector<Op>& second) {
  first.insert(first.end(), second.begin(), second.end());
  return first;
}

// Pushes `costs` in order, then pops until the queue is empty and once more.
Sequence fill_then_drain(const std::string& name, const std::string& about,
                         const std::vector<float>& costs) {
  return {name, about,
          concat(concat(pushes(costs), {peek()}), pops(static_cast<int>(costs.size()) + 1))};
}

// A 64-bit linear congruential generator (Knuth's MMIX constants), high bits out.
struct Lcg {
  std::uint64_t state;
  std::uint32_t next() {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<std::uint32_t>(state >> 33);
  }
  // A whole number in [0, bound).
  std::uint32_t below(std::uint32_t bound) { return next() % bound; }
};

// `length` steps drawn from `seed`: a push of a cost drawn from `costs` with the
// chance `push_percent`, else a peek one time in five, else a pop.
Sequence drawn(const std::string& name, const std::string& about, std::uint64_t seed,
               int length, int push_percent, const std::vector<float>& costs) {
  Lcg lcg{seed};
  std::vector<Op> ops;
  for (int step = 0; step < length; ++step) {
    if (static_cast<int>(lcg.below(100)) < push_percent) {
      ops.push_back(push(costs[lcg.below(static_cast<std::uint32_t>(costs.size()))]));
    } else if (lcg.below(5) == 0) {
      ops.push_back(peek());
    } else {
      ops.push_back(pop());
    }
  }
  return {name, about, ops};
}

// A heap grown by `grow` drawn steps at 75 percent pushes, then emptied by drawn
// steps at 25 percent pushes and a final drain.
Sequence grow_then_shrink(const std::string& name, const std::string& about,
                          std::uint64_t seed, int grow, int shrink,
                          const std::vector<float>& costs) {
  Sequence growing = drawn(name, about, seed, grow, 75, costs);
  Sequence shrinking = drawn(name, about, seed + 1, shrink, 25, costs);
  std::vector<Op> ops = concat(growing.ops, shrinking.ops);
  int size = 0;
  for (const Op& op : ops) {
    if (op.kind == Kind::Push) ++size;
    if (op.kind == Kind::Pop && size > 0) --size;
  }
  return {name, about, concat(ops, pops(size + 1))};
}

std::vector<Sequence> sequences() {
  std::vector<Sequence> all;
  all.push_back({"empty_queue",
                 "peek and pop on an empty queue give nothing; one entry goes in and comes out",
                 {peek(), pop(), push(3.0f), peek(), pop(), peek(), pop()}});
  for (int count = 1; count <= 12; ++count) {
    all.push_back(fill_then_drain("all_equal_" + std::to_string(count),
                                  std::to_string(count) + (count == 1 ? " push" : " pushes") +
                                      " of one cost, then a drain: every comparison "
                                      "is a tie, at odd and even lengths",
                                  std::vector<float>(count, 1.0f)));
  }
  all.push_back(fill_then_drain("ascending_pairs", "costs rising in equal pairs",
                                {1, 1, 2, 2, 3, 3, 4, 4, 5, 5}));
  all.push_back(fill_then_drain("descending_pairs", "costs falling in equal pairs",
                                {5, 5, 4, 4, 3, 3, 2, 2, 1, 1}));
  all.push_back(fill_then_drain("low_cost_last",
                                "equal costs, then one lower cost that rises to the top",
                                {2, 2, 2, 2, 2, 2, 2, 1}));
  all.push_back(fill_then_drain("two_costs_alternating", "two costs, alternating",
                                {1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1}));
  all.push_back(fill_then_drain("children_tie_parent_lower",
                                "two children of equal cost under a lower parent, at "
                                "several depths",
                                {0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3}));
  all.push_back({"interleaved_hand",
                 "pushes between pops, with ties on both sides of the pops",
                 {push(2), push(1), push(2), push(1), push(3), push(1), push(2), push(2),
                  push(1), pop(), push(1), peek(), push(2), pop(), pop(), push(1), push(1),
                  pop(), pop(), pop(), pop(), push(2), pop(), pop(), pop(), pop(), pop(),
                  pop(), pop()}});
  all.push_back(drawn("drawn_one_cost", "drawn steps, every push of one cost", 1, 60, 55, {1}));
  all.push_back(
      drawn("drawn_two_costs", "drawn steps, pushes of two costs", 2, 60, 55, {1, 2}));
  all.push_back(drawn("drawn_three_costs", "drawn steps, pushes of three costs", 3, 60, 55,
                      {0.5f, 1, 1.5f}));
  all.push_back(drawn("drawn_five_costs", "drawn steps, pushes of five costs", 4, 60, 55,
                      {0, 0.25f, 0.5f, 0.75f, 1}));
  all.push_back(grow_then_shrink("grow_shrink_two_costs",
                                 "a heap grown by drawn steps, mostly pushes, then "
                                 "shrunk by drawn steps and drained; two costs",
                                 5, 40, 40, {1, 2}));
  all.push_back(grow_then_shrink("grow_shrink_four_costs",
                                 "a heap grown by drawn steps, mostly pushes, then "
                                 "shrunk by drawn steps and drained; four costs",
                                 7, 40, 40, {3, 3.25f, 3.5f, 4}));
  all.push_back(fill_then_drain("signed_zero",
                                "0 and -0 are equal costs; each entry keeps its own sign",
                                {0.0f, -0.0f, 0.0f, -0.0f, -0.0f, 0.0f, 1.0f}));
  all.push_back(fill_then_drain("infinity",
                                "infinite costs tie with each other and go below finite ones",
                                {kInf, 1, kInf, 2, kInf, 1, kInf, -kInf}));
  all.push_back(fill_then_drain("nan_pushed_last",
                                "a NaN cost compares neither greater nor less than any cost",
                                {1, 2, 3, 1, 2, kNaN}));
  all.push_back(fill_then_drain("nan_pushed_first",
                                "a NaN cost compares neither greater nor less than any cost",
                                {kNaN, 3, 2, 1, 2, 1, 0}));
  all.push_back(fill_then_drain("nan_in_descent",
                                "a NaN child beside a child of finite cost in the descent of a pop",
                                {1, 1, kNaN, 1}));
  all.push_back(drawn("nan_drawn", "drawn steps, NaN among the pushed costs", 9, 50, 55,
                      {1, 2, kNaN}));
  return all;
}

}  // namespace

int main() {
  std::vector<Sequence> all = sequences();
  std::printf("{\"queue\": \"std::priority_queue<std::pair<float, int>, "
              "std::vector<std::pair<float, int>>, Cmp> with Cmp(a, b) = a.first > b.first\",\n");
  std::printf(" \"compiler\": \"g++ %s, libstdc++ %d\",\n", __VERSION__, __GLIBCXX__);
  std::printf(" \"vectors\": [\n");
  for (std::size_t index = 0; index < all.size(); ++index) {
    std::printf("%s%s\n", run_json(all[index]).c_str(), index + 1 < all.size() ? "," : "");
  }
  std::printf("]}\n");
  return 0;
}
