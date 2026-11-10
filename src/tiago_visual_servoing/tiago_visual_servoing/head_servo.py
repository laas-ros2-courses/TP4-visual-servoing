import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, JointState
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

PAN_JOINT = 'head_1_joint'
TILT_JOINT = 'head_2_joint'
PAN_LIMITS = (math.radians(-75.0), math.radians(75.0))
TILT_LIMITS = (math.radians(-60.0), math.radians(45.0))

KP_PAN = -0.0025   # confirmed converging (sandbox + real-repo test)
KP_TILT = -0.0025
DEADZONE_PX = 6.0
MAX_STEP_RAD = math.radians(3.0)


class HeadServo(Node):
    def __init__(self):
        super().__init__('head_servo')
        self.cx = None
        self.cy = None
        self.pan = 0.0
        self.tilt = 0.0

        self.create_subscription(CameraInfo, '/head_front_camera/camera_info', self.camera_info_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(PointStamped, '/aruco_tracker/target_pixel', self.target_cb, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)

    def camera_info_cb(self, msg):
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def joint_state_cb(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name == PAN_JOINT:
                self.pan = position
            elif name == TILT_JOINT:
                self.tilt = position

    def target_cb(self, msg):
        if self.cx is None:
            return

        du = msg.point.x - self.cx
        dv = msg.point.y - self.cy

        if abs(du) < DEADZONE_PX and abs(dv) < DEADZONE_PX:
            return

        dpan = KP_PAN * du
        dtilt = KP_TILT * dv

        dpan = max(-MAX_STEP_RAD, min(MAX_STEP_RAD, dpan))
        dtilt = max(-MAX_STEP_RAD, min(MAX_STEP_RAD, dtilt))

        pan_target = max(PAN_LIMITS[0], min(PAN_LIMITS[1], self.pan + dpan))
        tilt_target = max(TILT_LIMITS[0], min(TILT_LIMITS[1], self.tilt + dtilt))

        self.send_trajectory(pan_target, tilt_target)

    def send_trajectory(self, pan, tilt):
        traj = JointTrajectory()
        traj.joint_names = [PAN_JOINT, TILT_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = Duration(sec=0, nanosec=150_000_000)
        traj.points = [point]
        self.traj_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = HeadServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
