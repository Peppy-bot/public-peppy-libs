# libstdcxx_heap

`OpenSet`, a priority queue of entries of an `f32` cost and an item, that gives its
entries in the order of libstdc++'s `std::priority_queue` (GCC 15) with a comparator
that returns `a.first > b.first`. The entry of least cost comes out first. Entries of
equal cost come out in the order that libstdc++'s queue gives them, which depends on the
slot of each entry in its binary heap. A port of a C++ search that takes its ties from
such a queue, such as Nav2's Smac 2D planner, takes its entries in the same order through
`OpenSet`.

The queue is a binary heap in a vector, its entry of least cost in the first slot. A push
puts the entry in a new last slot and moves it up past each parent of greater cost. A pop
takes out the first entry, moves the empty first slot down to a leaf along the child of
lesser cost, the right child when the two costs are equal, and moves the last entry up
from that leaf past each parent of greater cost: Floyd's variant of the deletion from a
heap (Knuth, The Art of Computer Programming, vol. 3, section 5.2.3). Costs compare with
`>`, so a NaN cost is neither greater nor less than another cost.

- `OpenSet::push` adds an entry, as `priority_queue::push` does.
- `OpenSet::pop` takes out the entry that comes out next and gives it, as `top` followed
  by `pop` does.

No dependencies. The tests check each push and pop, and the heap after each, against what
`std::priority_queue` of GCC 15 gave for the same calls.
