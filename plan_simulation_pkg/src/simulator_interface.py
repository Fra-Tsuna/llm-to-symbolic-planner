#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String
from plan_simulation_pkg.msg import StringList
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

SIM_ACTIONS_TOPIC = "/canopies_simulator/simulator/sim_action"
SIM_2_PARSER_ACK_TOPIC = "/canopies_simulator/orchestrator/sim_2_pars_ack"
ODOMETRY_TOPIC = "/canopies_simulator/moving_base/odometry"
TELEPORT_TOPIC = "/canopies_simulator/moving_base/teleport"
TWIST_TOPIC = "/canopies_simulator/moving_base/twist"


def rot2quat(R):
    qw = np.sqrt(1 + np.trace(R)) / 2
    qx = (R[2, 1] - R[1, 2]) / (4 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4 * qw)
    return qx, qy, qz, qw


class SimInterface:
    def __init__(self):
        rospy.init_node("sim_interface", anonymous=True)
        self.ack_publisher = rospy.Publisher(
            SIM_2_PARSER_ACK_TOPIC, String, queue_size=100
        )
        self.pose_teleport_publisher = rospy.Publisher(
            TELEPORT_TOPIC, Pose, queue_size=100
        )
        self.twist_publisher = rospy.Publisher(TWIST_TOPIC, Twist, queue_size=100)
        self.sim_actions_subscriber = rospy.Subscriber(
            SIM_ACTIONS_TOPIC, StringList, self.callback
        )

        rospy.loginfo("Sim Interface Node Initialized")

        self.initial_pose = Pose()
        self.set_initial_pose()
        self.twist_rate = rospy.Rate(10)

        self.location_mapper_dict = {
            "l0": [1.8226, 1.0145],
            "l1": [1.8226, -1.0384],
            "l2": [1.8226, -4.0730],
            "l3": [1.8226, -6.9446],
        }

        self.action_mapper_dict = {
            "move": self.move,
            "call_support": self.call_support,
            "check_grape": self.check_grape,
            "harvest_grape": self.harvest_grape,
            "drop_grape": self.drop_grape,
            "handle_exception": self.handle_exception,
            "assest_vine": self.assest_vine,
        }

    def set_initial_pose(self):
        spawn_pose_msg = rospy.wait_for_message(ODOMETRY_TOPIC, Odometry)
        self.initial_pose.position.x = 1.8226
        self.initial_pose.position.y = 1.0145
        self.initial_pose.position.z = spawn_pose_msg.pose.pose.position.z+0.1
        qx, qy, qz, qw = rot2quat(np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]))
        self.initial_pose.orientation.x = qx
        self.initial_pose.orientation.y = qy
        self.initial_pose.orientation.z = qz
        self.initial_pose.orientation.w = -qw
        rospy.sleep(1)
        self.pose_teleport_publisher.publish(self.initial_pose)
        return

    def run(self):
        rospy.spin()

    def callback(self, sim_action):

        action_type = sim_action.data[0]
        action_params = sim_action.data[1:]

        if action_type in self.action_mapper_dict:
            self.action_mapper_dict[action_type](action_params)
        else:
            rospy.logerr(f"Action {action_type} not found in action_mapper")

        rospy.loginfo(f"MOSSO")

        ack_msg = String("EXECUTED")
        rospy.sleep(1)
        self.ack_publisher.publish(ack_msg)

    def move(self, action_args, speed=0.5, tolerance=0.05):
        from_location, to_location = action_args
        from_location = self.location_mapper_dict[from_location]
        to_location = self.location_mapper_dict[to_location]

        twist = Twist()

        dx = from_location[0] - to_location[0]
        dy = from_location[1] - to_location[1]
        distance = np.sqrt(dx**2 + dy**2)
        while distance > tolerance:
            print(from_location)
            twist.linear.x = min(speed, distance)
            self.twist_publisher.publish(twist)
            self.twist_rate.sleep()
            odom = rospy.wait_for_message(ODOMETRY_TOPIC, Odometry)
            from_location = [odom.pose.pose.position.x, odom.pose.pose.position.y]
            dx = from_location[0] - to_location[0]
            dy = from_location[1] - to_location[1]
            distance = np.sqrt(dx**2 + dy**2)

        self.twist_publisher.publish(Twist())
        return

    def call_support(self, action_args):
        pass

    def check_grape(self, action_args):
        pass

    def harvest_grape(self, action_args):
        pass

    def drop_grape(self, action_args):
        pass

    def handle_exception(self, action_args):
        pass

    def assest_vine(self, action_args):
        pass


if __name__ == "__main__":
    try:
        interface = SimInterface()
        interface.run()
    except rospy.ROSInterruptException:
        pass
    rospy.is_shutdown()
