#pragma once

#include "rust/cxx.h"

namespace ompl_bridge {

struct PlanRequest;
struct PlanResult;
struct Callbacks;

// Plans one request with OMPL, calling the caller's checks and constraint back
// through `callbacks`. One plan runs at a time in a process.
PlanResult plan(const PlanRequest& request, const Callbacks& callbacks);

}  // namespace ompl_bridge
