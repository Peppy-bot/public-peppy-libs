"""brain_core: the code every brain node shares.

A brain serves item_perception and item_manipulation over a robot that
limb_motion drives and a camera that rgbd_camera streams. brain_core holds
the contract rules, the robot proxy, the camera geometry and the goal
loops; a node brings its generated modules (`Bindings`) and its backends,
a `Detector` and a `Manipulator`, and calls `node.setup`.
"""
