#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from plan_simulation_pkg.msg import StringList


PDDL_ACTIONS_TOPIC = "/canopies_simulator/orchestrator/pddl_action"
SIM_ACTIONS_TOPIC = "/canopies_simulator/simulator/sim_action"
SIM_2_PARSER_ACK_TOPIC = "/canopies_simulator/orchestrator/sim_2_pars_ack"
PARSER_2_ORCH_ACK_TOPIC = "/canopies_simulator/orchestrator/pars_2_orch_ack"

GREY = "\033[90m"  # Grey for logdebug
GREEN = "\033[92m"  # Green for loginfo
YELLOW = "\033[93m"  # Yellow for logwarn
RED = "\033[91m"  # Red for logerr
RESET = "\033[0m"  # Reset to default color


class Parser:
    def __init__(self):
        rospy.init_node("parser", anonymous=True)
        # self.simulator_publisher = rospy.Publisher(..., ..., queue_size=100)
        self.orchestrator_ack_publisher = rospy.Publisher(
            PARSER_2_ORCH_ACK_TOPIC, String, queue_size=100
        )
        self.simulator_actions_publisher = rospy.Publisher(
            SIM_ACTIONS_TOPIC, StringList, queue_size=100
        )
        rospy.Subscriber(PDDL_ACTIONS_TOPIC, String, self.callback)
        rospy.loginfo(f"{GREEN}Parser Node Initialized{RESET}")

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

    def callback(self, action_msg):
        action = action_msg.data
        rospy.loginfo(f"{GREY}Received action: {action}{RESET}")
        sim_action = self.action_mapper(action)
        rospy.sleep(1)
        self.simulator_actions_publisher.publish(sim_action)

        sim_ack_msg = rospy.wait_for_message(SIM_2_PARSER_ACK_TOPIC, String)
        if sim_ack_msg.data == "EXECUTED":
            rospy.loginfo(f"{GREY}Received ack: action executed{RESET}")
        else:
            rospy.loginfo(f"{GREY}User question: {sim_ack_msg.data}{RESET}")

        orch_ack_msg = sim_ack_msg

        rospy.sleep(1)
        self.orchestrator_ack_publisher.publish(orch_ack_msg)

    def run(self):
        rospy.spin()

    def action_mapper(self, pddl_action):
        pddl_action = pddl_action.replace("(", "").replace(")", "")
        action_name = pddl_action.split(" ")[0]
        action_args = pddl_action.split(" ")[1:]

        if action_name in self.action_mapper_dict:
            sim_action = self.action_mapper_dict[action_name](action_args)
            return sim_action
        else:
            rospy.logerr(f"{RED}Action {action_name} not found in action_mapper{RESET}")

    def move(self, action_args):
        _, from_location, to_location = action_args
        action_msg = StringList()
        action_msg.data = ["move", from_location, to_location]
        return action_msg

    def call_support(self, action_args):
        _, location, support = action_args
        action_msg = StringList()
        action_msg.data = ["call_support", location, support]
        return action_msg

    def check_grape(self, action_args):
        _, grape, location = action_args
        action_msg = StringList()
        action_msg.data = ["check_grape", grape, location]
        return action_msg

    def harvest_grape(self, action_args):
        _, grape, box, location = action_args
        action_msg = StringList()
        action_msg.data = ["harvest_grape", grape, box, location]
        return action_msg

    def drop_grape(self, action_args):
        pass
    
    def empty_box(self, action_args):
        pass

    def handle_exception(self, action_args):
        _, location = action_args
        action_msg = StringList()
        action_msg.data = ["handle_exception", location]
        return action_msg

    def assest_vine(self, action_args):
        _, grape, location = action_args
        action_msg = StringList()
        action_msg.data = ["assest_vine", grape, location]
        return action_msg


if __name__ == "__main__":
    try:
        parser = Parser()
        parser.run()
    except rospy.ROSInterruptException:
        pass
