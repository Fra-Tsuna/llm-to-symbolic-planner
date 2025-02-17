import json
import random

from GPT_Agents import *


INIT_STATE = [
    "(robot-at rob l0)",
    "(support support0)",
    "(grape-at g0 l0)",
    "(grape-at g1 l1)",
    "(grape-at g2 l2)",
    "(grape-at g3 l3)",
    "(unchecked l0)",
    "(unchecked l1)",
    "(unchecked l2)",
    "(unchecked l3)",
    "(free rob)",
    "(in b rob)",
    "(adj l0 l1)",
    "(adj l1 l2)",
    "(adj l2 l3)",
    "(full b)",
]

PLAN_PATH = "config/PDDL/sas_plan_adapted"

with open("config/action_schemas.json", "r") as file:
    ACTIONS_SCHEMA = json.load(file)


def process_action(action):
    # split name and arguments
    action = action.replace("(", "").replace(")", "")
    action = action.split(" ")
    action = {
        "name": action[0],
        "args": action[1:],
    }

    # print("ACTION ", action)

    action_schema = ACTIONS_SCHEMA[action["name"]]
    add_set = action_schema["add_set"].split(",")
    del_set = action_schema["del_set"].split(",")
    for i, arg in enumerate(action["args"]):
        add_set = [x.replace(f"?{i+1}", arg) for x in add_set]
        del_set = [x.replace(f"?{i+1}", arg) for x in del_set]

    return action, set(add_set), set(del_set)


def add_fluents(state, add_set):
    for fluent in add_set:
        if fluent not in state:
            state.add(fluent)


def remove_fluents(state, del_set):
    for fluent in del_set:
        if fluent in state:
            state.remove(fluent)


def get_next_state(state, action):
    action, add_set, del_set = process_action(action)

    add_fluents(state, add_set)
    remove_fluents(state, del_set)

    return state


def get_current_state(plan_so_far):
    state = INIT_STATE.copy()
    state = set(state)

    for action in plan_so_far:
        get_next_state(state, action)

    return [state]


def get_future_states(plan_so_far, plan):
    state = INIT_STATE.copy()
    state = set(state)

    for action in plan_so_far:
        get_next_state(state, action)

    states = [state]
    last_action = plan_so_far[-1]
    last_action_index = plan.index(last_action)

    for action in plan[last_action_index + 1 :]:
        get_next_state(state, action)
        states.append(state)

    return states


def get_past_states(plan_so_far):
    state = INIT_STATE.copy()
    state = set(state)

    states = [state]
    for action in plan_so_far:
        get_next_state(state, action)
        states.append(state)

    return states


def evaluate_metric(real_states, predicted_state):
    results = 0.0

    for real_state in real_states:
        count = 0

        for fluent in predicted_state:
            if fluent in real_state:
                count += 1

        results += count / len(predicted_state)

    return results / len(real_states)


def simulate_plan(plan, question, question_probability=0.25):
    p = question_probability
    increment = (1 - p) / len(plan)
    plan_so_far = []
    psf_returned = []
    system_response = []
    for i, action in enumerate(plan):
        plan_so_far.append(f"{i+1}) {action}")
        psf_returned.append(action)

        if random.random() < p:
            user_response = question
            chat = GPTChat()
            system_response = chat(plan_so_far, user_response)
            break
        else:
            p += increment
            p = min(1, p)

    return system_response, psf_returned


def load_plan(plan_path):
    plan = []
    with open(plan_path, "r") as file:
        temp = file.readlines()
        plan.extend(line.replace("\n", "") for line in temp)
    return plan
