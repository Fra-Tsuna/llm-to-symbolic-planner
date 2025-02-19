#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from plan_simulation_pkg.msg import StringList


PDDL_ACTIONS_TOPIC = "/canopies_simulator/orchestrator/pddl_action"
SIM_ACTIONS_TOPIC = "/canopies_simulator/simulator/sim_action"
SIM_2_PARSER_ACK_TOPIC = "/canopies_simulator/orchestrator/sim_2_pars_ack"
PARSER_2_ORCH_ACK_TOPIC = "/canopies_simulator/orchestrator/pars_2_orch_ack"


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
        rospy.loginfo("Parser Node Initialized")

        self.action_mapper_dict = {
            "move": self.move,
            "call_support": self.call_support,
            "check_grape": self.check_grape,
            "harvest_grape": self.harvest_grape,
            "drop_grape": self.drop_grape,
            "handle_exception": self.handle_exception,
            "assest_vine": self.assest_vine,
        }

    def callback(self, action_msg):
        action = action_msg.data
        rospy.loginfo(f"Received action: {action}")
        sim_action = self.action_mapper(action)
        rospy.sleep(1)
        self.simulator_actions_publisher.publish(sim_action)

        ack_msg = rospy.wait_for_message(SIM_2_PARSER_ACK_TOPIC, String)
        rospy.loginfo(f"Received ack: {ack_msg.data}")
        # TODO insert code here to parse the action and send it to the sim####
        # TODO receive from the simulator the ack of action executed

        # if not interrupt:
        #     ack_msg = String()
        #     ack_msg.data = "EXECUTED"
        # else:
        #     ack_msg = String()
        #     ack_msg.data = ...

        rospy.sleep(1)
        self.orchestrator_ack_publisher.publish(ack_msg)

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
            rospy.logerr(f"Action {action_name} not found in action_mapper")

    def move(self, action_args):
        _, from_location, to_location = action_args
        action_msg = StringList()
        action_msg.data = ["move", from_location, to_location]
        return action_msg

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
        parser = Parser()
        parser.run()
    except rospy.ROSInterruptException:
        pass
