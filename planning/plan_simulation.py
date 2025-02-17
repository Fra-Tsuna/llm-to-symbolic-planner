import random
import select
import sys

import rich
from GPT_Agents import FluentsExtractor, GPTChat
from process_states import evaluate_metric, get_current_state, load_plan


PLAN_PATH = "config/PDDL/sas_plan_adapted"
WAITING_TIME = 10.0


def get_user_input(prompt, timeout=1):
    console = rich.console.Console()
    console.print(prompt)

    inputs, _, _ = select.select([sys.stdin], [], [], timeout)

    return sys.stdin.readline().strip() if inputs else None


def test_simulate_plan(plan, waiting_time):
    console = rich.console.Console()
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
            chat = GPTChat()
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

    # extractor = FluentsExtractor()

    # fluents = extractor(system_response)
    # fluents = fluents.split(",")

    # current_state = get_current_state(plan_so_far)

    # print("FLUENTS:")
    # for fluent in fluents:
    #     print(f"{fluent}\n")

    # print("CURRENT STATE:")
    # for fluent in current_state:
    #     print(f"{fluent}\n")

    # print(evaluate_metric(current_state, fluents))
