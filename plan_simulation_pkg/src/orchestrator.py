#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from GPT_Agents import GPTChat
import os

ROOT_DIR = os.path.abspath(__file__ + "/../..")
PLAN_PATH = ROOT_DIR + "/config/PDDL/sas_plan_adapted"

domain = []
DOMAIN_PATH = ROOT_DIR + "/config/PDDL/domain.pddl"
with open(DOMAIN_PATH, "r") as file:
    domain = file.read()

problem = []
PROBLEM_PATH = ROOT_DIR + "/config/PDDL/problem.pddl"
with open(PROBLEM_PATH, "r") as file:
    problem = file.read()

human_policy = []
POLICY_PATH = ROOT_DIR + "/config/PDDL/human_policy.pol"
with open(POLICY_PATH, "r") as file:
    human_policy = file.read()

PDDL_ACTIONS_TOPIC = "/canopies_simulator/orchestrator/pddl_action"
PARSER_2_ORCH_ACK_TOPIC = "/canopies_simulator/orchestrator/pars_2_orch_ack"


def load_plan(plan_path):
    plan = []
    with open(plan_path, "r") as file:
        temp = file.readlines()
        plan.extend(line.replace("\n", "") for line in temp)
    return plan


class Orchestrator:
    def __init__(self):
        rospy.init_node("orchestrator", anonymous=True)
        self.action_publisher = rospy.Publisher(
            PDDL_ACTIONS_TOPIC, String, queue_size=100
        )
        rospy.loginfo("Orchestrator Node Initialized")
        self.plan = load_plan(PLAN_PATH)
        chat_kwargs = {
            "domain": domain,
            "problem": problem,
            "human_policy": human_policy,
        }
        self.chat = GPTChat(**chat_kwargs)

    def run(self):
        plan_so_far = []

        for i, action in enumerate(self.plan):
            plan_so_far.append(f"{i+1}) {action}")
            action_msg = String()
            action_msg.data = action
            rospy.sleep(1)
            self.action_publisher.publish(action_msg)
            ack_msg = rospy.wait_for_message(PARSER_2_ORCH_ACK_TOPIC, String)
            if ack_msg.data == "EXECUTED":
                rospy.loginfo(f"Action {action} executed")
            else:
                rospy.loginfo(f"User question: {ack_msg.data}")
                user_response = ack_msg.data
                system_response = self.chat(plan_so_far, user_response)
                print(f"Response: {system_response}")
                break


if __name__ == "__main__":
    try:
        orchestrator = Orchestrator()
        orchestrator.run()
    except rospy.ROSInterruptException:
        pass
    rospy.is_shutdown()
