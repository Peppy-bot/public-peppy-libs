"""What a simulation engine node knows about the robots it stands, whatever
its physics: the registry of standing robots (registry), the six pairing
slots and what each pair names (pairs), one entry per robot model with its
checks (models), the setpoints an engine takes on a pair (setpoints), each
object's and robot's record on object_state and a snapshot of both
(object_state), the rule a MuJoCo model of a base's wheel is held to
(rate_loop), the camera configuration (cameras), the lidar configuration
(lidars), the one frame of a
simulation and the commands given in it (world), and the error of an entry
that does not read or fit (errors)."""
