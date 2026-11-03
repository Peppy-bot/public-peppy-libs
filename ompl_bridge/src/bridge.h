#pragma once

#include "rust/cxx.h"

namespace ompl_bridge {

struct PlanRequest;
struct PlanResult;
struct Callbacks;

// Plans one request with OMPL, calling the caller's checks and constraint back
// through `callbacks`: on the calling thread for a request of one attempt, and on a
// thread of each attempt besides for a request of several.
PlanResult plan(const PlanRequest& request, const Callbacks& callbacks);

}  // namespace ompl_bridge
