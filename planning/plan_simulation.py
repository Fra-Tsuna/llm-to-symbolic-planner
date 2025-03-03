import select
import sys

import rich
from GPT_Agents import GPTChat
from process_states import load_plan

console = rich.console.Console()

PLAN_PATH = "config/PDDL/sas_plan_adapted"
WAITING_TIME = 10.0

domain = []
DOMAIN_PATH = "config/PDDL/domain.pddl"
with open(DOMAIN_PATH, "r") as file:
    domain = file.read()

problem = []
PROBLEM_PATH = "config/PDDL/problem.pddl"
with open(PROBLEM_PATH, "r") as file:
    problem = file.read()

human_policy = []
POLICY_PATH = "config/PDDL/human_policy.pol"
with open(POLICY_PATH, "r") as file:
    human_policy = file.read()

chat_kwargs = {"domain": domain, "problem": problem, "human_policy": human_policy}

def get_user_input(prompt, timeout=1):
    console = rich.console.Console()
    console.print(prompt)

    inputs, _, _ = select.select([sys.stdin], [], [], timeout)

    return sys.stdin.readline().strip() if inputs else None


def test_simulate_plan(plan, waiting_time):
    plan_so_far = []
    psf_returned = []
    system_response = []
    for i, action in enumerate(plan):
        plan_so_far.append(f"{i+1}) {action}")
        psf_returned.append(action)
        console.print(f"[bold red]ROBOT: [/bold red]Action: {action} executed.")
        user_response = get_user_input(
            "[bold yellow]Do you want to ask anything?[/bold yellow]\n[bold green]USER: [/bold green]",
            waiting_time,
        )

        if user_response:
            chat = GPTChat(**chat_kwargs)
            system_response = chat(plan_so_far, user_response)
            console.print(f"[bold red]ROBOT: [/bold red] {system_response}")
            print("--------------------------------------------------------")
            break
    return system_response, psf_returned


if __name__ == "__main__":
    plan_path = PLAN_PATH
    waiting_time = WAITING_TIME
    plan = load_plan(plan_path)
    system_response, plan_so_far = test_simulate_plan(plan, waiting_time)
    console.print(f"[bold green]SYSTEM: [/bold green] {system_response}")