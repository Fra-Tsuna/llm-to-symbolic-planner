#!/usr/bin/env python3

import rospy
import numpy as np
import subprocess

from std_msgs.msg import String, Float64MultiArray
from plan_simulation_pkg.msg import StringList
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from canopies_simulator.srv import teleport_human, teleport_humanRequest, BEM

SIM_ACTIONS_TOPIC = "/canopies_simulator/simulator/sim_action"
SIM_2_PARSER_ACK_TOPIC = "/canopies_simulator/orchestrator/sim_2_pars_ack"

ODOMETRY_TOPIC = "/canopies_simulator/farming/moving_base/odometry"
TELEPORT_TOPIC = "/canopies_simulator/farming/moving_base/teleport"
TWIST_TOPIC = "/canopies_simulator/farming/moving_base/twist"

CONTROLLER_VELOCITIES_TOPIC = (
    "canopies_simulator/farming/joint_group_velocity_controller/command"
)

HUMAN_TELEPORT_SERVICE = "/canopies_simulator/human00/human_teleport"

SUPPORT_TELEPORT_TOPIC = "/canopies_simulator/logistics/moving_base/teleport"

BEM_ROBOT_SERVICE = "/canopies_simulator/farming/BEM_service"
BEM_SUPPORT_SERVICE = "/canopies_simulator/logistics/BEM_service"

GREY = "\033[90m"  # Grey for logdebug
GREEN = "\033[92m"  # Green for loginfo
YELLOW = "\033[93m"  # Yellow for logwarn
RED = "\033[91m"  # Red for logerr
RESET = "\033[0m"  # Reset to default color


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
        self.support_pose_teleport_publisher = rospy.Publisher(
            SUPPORT_TELEPORT_TOPIC, Pose, queue_size=100
        )
        self.twist_publisher = rospy.Publisher(TWIST_TOPIC, Twist, queue_size=100)
        self.sim_actions_subscriber = rospy.Subscriber(
            SIM_ACTIONS_TOPIC, StringList, self.callback
        )
        self.robot_manipulator_vel_publisher = rospy.Publisher(
            CONTROLLER_VELOCITIES_TOPIC,
            Float64MultiArray,
            queue_size=1,
        )
        self.human_pose_teleport_srv_proxy = rospy.ServiceProxy(
            HUMAN_TELEPORT_SERVICE, teleport_human
        )
        self.robot_bem_service = rospy.ServiceProxy(BEM_ROBOT_SERVICE, BEM)
        self.support_bem_service = rospy.ServiceProxy(BEM_SUPPORT_SERVICE, BEM)

        rospy.loginfo(GREEN + "Sim Interface Node Initialized" + RESET)

        self.initial_pose = Pose()
        self.reset_initial_pose()
        self.support_pose = Pose()
        self.reset_support_robot()
        self.human_pose = Pose()
        self.reset_human()
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
            "empty_box": self.empty_box,
        }

    def run(self):
        rospy.spin()

    def callback(self, sim_action):

        action_type = sim_action.data[0]
        action_params = sim_action.data[1:]

        if action_type in self.action_mapper_dict:
            rospy.loginfo(f"{GREY}Executing action: {action_type}{RESET}")
            self.action_mapper_dict[action_type](action_params)
        else:
            rospy.logerr(f"{RED}Action {action_type} not found in action_mapper{RESET}")

        human_question = input(f"{YELLOW}User:{RESET} ")
        ack = human_question if human_question else "EXECUTED"

        ack_msg = String(ack)
        rospy.sleep(1)
        self.ack_publisher.publish(ack_msg)

    def reset_initial_pose(self):
        spawn_pose_msg = rospy.wait_for_message(ODOMETRY_TOPIC, Odometry)
        self.initial_pose.position.x = 1.8226
        self.initial_pose.position.y = 1.0145
        self.initial_pose.position.z = spawn_pose_msg.pose.pose.position.z + 0.1
        qx, qy, qz, qw = rot2quat(np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]))
        self.initial_pose.orientation.x = qx
        self.initial_pose.orientation.y = qy
        self.initial_pose.orientation.z = qz
        self.initial_pose.orientation.w = -qw
        rospy.sleep(1)
        self.pose_teleport_publisher.publish(self.initial_pose)
        self.init_parameter_server("torso")
        self.do_control_loop([0.0], 3)
        self.init_parameter_server("head")
        self.do_control_loop([0.0, 0.0], 50)
        return

    def reset_human(self, id=0):
        rospy.wait_for_service(HUMAN_TELEPORT_SERVICE)

        request = teleport_humanRequest()
        request.human_id = 0
        request.pose.position.x = -100.0
        request.pose.position.y = 100.0
        request.pose.position.z = 0.0
        request.pose.orientation.x = 0.0
        request.pose.orientation.y = 0.0
        request.pose.orientation.z = 0.0
        request.pose.orientation.w = 1.0
        self.human_id = id
        self.human_pose = request.pose

        _ = self.human_pose_teleport_srv_proxy(request)
        return

    def reset_support_robot(self):
        self.support_pose = Pose()
        self.support_pose.position.x = 100.0
        self.support_pose.position.y = 100.0
        self.support_pose.position.z = 0.18868
        self.support_pose.orientation = self.initial_pose.orientation
        rospy.sleep(1)
        self.support_pose_teleport_publisher.publish(self.support_pose)
        return

    def init_parameter_server(self, _name: str):

        if _name == "right" or _name == "left":
            self.names = [
                f"arm_{_name}_1_joint",
                f"arm_{_name}_2_joint",
                f"arm_{_name}_3_joint",
                f"arm_{_name}_4_joint",
                f"arm_{_name}_5_joint",
                f"arm_{_name}_6_joint",
                f"arm_{_name}_7_joint",
            ]
        elif _name == "head":
            self.names = [
                "head_1_joint",
                "head_2_joint",
            ]
        elif _name == "torso":
            self.names = [
                "torso_lift_joint",
            ]
        else:
            rospy.logerr(f"{RED}Invalid name{_name}{RESET}")

        rospy.set_param(
            "canopies_simulator/farming/joint_group_velocity_controller/joints",
            self.names,
        )
        return

    def do_control_loop(self, positions, ctrl_vel_param=50):
        control_loop_rate = rospy.Rate(3)  # 10Hz
        control_loop_time = 2  # Sec

        now = rospy.Time.now()
        while rospy.Time.now() < now + rospy.Duration.from_sec(control_loop_time):
            joint_state = rospy.wait_for_message(
                "canopies_simulator/farming/joint_states", JointState, 10
            )

            # Calculation of new velocities for every joint
            velocity_msg = Float64MultiArray()
            velocity_msg.data = [0.0] * len(positions)
            for name in self.names:
                index_in_joint_state = joint_state.name.index(name)
                index_in_msg = self.names.index(name)
                final_pos = positions[index_in_msg]
                real_pos = joint_state.position[index_in_joint_state]
                control_velocity = ctrl_vel_param * (final_pos - real_pos)
                velocity_msg.data[index_in_msg] = control_velocity

            self.robot_manipulator_vel_publisher.publish(velocity_msg)
            control_loop_rate.sleep()

        # Stop the joints
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [0.0] * len(positions)
        self.robot_manipulator_vel_publisher.publish(velocity_msg)

    def move(self, action_args, speed=0.5, tolerance=0.05):
        from_location, to_location = action_args
        from_location = self.location_mapper_dict[from_location]
        to_location = self.location_mapper_dict[to_location]

        twist = Twist()

        dx = from_location[0] - to_location[0]
        dy = from_location[1] - to_location[1]
        distance = np.sqrt(dx**2 + dy**2)
        while distance > tolerance:
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
        _ = action_args
        robot_pose = rospy.wait_for_message(ODOMETRY_TOPIC, Odometry)
        self.support_pose.position.x = robot_pose.pose.pose.position.x
        self.support_pose.position.y = robot_pose.pose.pose.position.y - 1.65
        self.support_pose.position.z = robot_pose.pose.pose.position.z
        self.support_pose.orientation.w *= -1

        self.support_pose_teleport_publisher.publish(self.support_pose)
        return

    def check_grape(self, action_args):
        _ = action_args

        self.init_parameter_server("head")
        self.do_control_loop([0.0, 0.3])
        self.do_control_loop([-0.7, 0.3])
        self.do_control_loop([0.7, 0.3])
        self.do_control_loop([0.0, 0.0])
        return

    def harvest_grape(self, action_args):
        _ = action_args

        self.init_parameter_server("right")
        self.do_control_loop([0.78, 1.4679, 1.143, 1.7095, 0.0, 1.3898, 0.0], 100)
        self.do_control_loop([1.2, -1.043, -1.0, -1.243, 1.0, 1.25, 1.0], 100)
        return

    def drop_grape(self, action_args):
        _ = action_args

        self.init_parameter_server("right")
        self.do_control_loop([0.9, 1.043, -1.0, -1.443, -1.24, -0.47, 1.0], 100)
        self.do_control_loop([-0.98, 1.4679, 1.143, 1.7095, 0.0, 1.3898, 0.0], 100)

    def empty_box(self, action_args):
        _ = action_args

        # had to do like this cause os simulator's limitations
        command = "rosservice call /canopies_simulator/logistics/BEM_service \"action: 'exchange'\" & rosservice call /canopies_simulator/farming/BEM_service \"action: 'exchange'\""
        subprocess.Popen(
            command, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        rospy.sleep(6)

        # move support to initial position
        self.support_pose = Pose()
        self.support_pose.position.x = 100.0
        self.support_pose.position.y = 100.0
        self.support_pose.position.z = 0.18868
        self.support_pose.orientation = self.initial_pose.orientation
        rospy.sleep(1)
        self.support_pose_teleport_publisher.publish(self.support_pose)

        return

    def handle_exception(self, action_args):
        location = action_args[0]
        location = self.location_mapper_dict[location]
        self.human_pose.position.x = location[0] - 0.75
        self.human_pose.position.y = location[1]
        request = teleport_humanRequest()
        request.human_id = self.human_id
        request.pose = self.human_pose
        self.human_pose_teleport_srv_proxy(request)
        return

    def assest_vine(self, action_args):
        _ = action_args

        self.init_parameter_server("torso")
        self.do_control_loop([0.3], 0.8)
        rospy.sleep(2)
        self.do_control_loop([0.0], 0.8)

        return


if __name__ == "__main__":
    try:
        interface = SimInterface()
        interface.run()
    except rospy.ROSInterruptException:
        pass
    rospy.is_shutdown()
