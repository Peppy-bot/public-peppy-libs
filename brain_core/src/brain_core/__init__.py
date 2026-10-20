"""brain_core: the code every brain node shares.

A brain serves item_perception, item_manipulation and brain_model over a
robot that limb_motion drives and a camera that rgbd_camera streams. brain_core holds
the contract rules, the robot proxy, the camera geometry and the goal
loops; a node brings its generated modules (`Bindings`) and its models,
a `ModelSource`, and calls `node.setup`.
"""
