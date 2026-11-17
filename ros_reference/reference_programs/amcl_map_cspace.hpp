// The reference program's copy of map_update_cspace of Nav2 1.5.2's AMCL, which
// amcl_map_cspace.cpp describes.
#pragma once

namespace ros_reference {

// How many times AMCL called the program's map_update_cspace, so a run can check that the
// dynamic linker bound AMCL's calls to it.
inline int amcl_map_cspace_updates = 0;

}  // namespace ros_reference
