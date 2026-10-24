# One step of a motion sequence executed by moveit_master.

uint8 ARM_POSE=0         # move the arm TCP to `pose` (planning frame, e.g. world)
uint8 ARM_NAMED=1        # move arm to SRDF named state `named_target` (e.g. "left_home")
uint8 GRIPPER_NAMED=2    # move gripper to SRDF named state `named_target` (e.g. "left_gripper_open")
uint8 WAIT=3             # pause for `duration` seconds
uint8 GRIPPER_JOINT=4    # move the gripper's first active joint (Robotiq knuckle) to `joint_value` (rad)
uint8 ARM_CARTESIAN=5    # straight-line TCP move by `offset` (m) in `frame`, e.g. z=-0.06 = straight down
uint8 ATTACH=6           # attach world object `object_id` to the arm's TCP link; its gripper links may touch it
uint8 DETACH=7           # detach `object_id`; it stays in the world where it was released
uint8 ALLOW_COLLISION=8  # allow=true: `object_id` may touch the arm's gripper links + `touch_links`; false: undo

uint8 type
string arm               # which arm runs the step ("left", "right"); "" = moveit_master's default_arm
string label             # free text for logs and error messages, e.g. "pick: descend"
geometry_msgs/Pose pose
string named_target
float64 duration
float64 joint_value
geometry_msgs/Vector3 offset
string frame             # ARM_CARTESIAN: "" = planning frame (world), "tcp" = TCP axes
string object_id
string[] touch_links     # ALLOW_COLLISION: extra links / objects, e.g. the table under a packet
bool allow
