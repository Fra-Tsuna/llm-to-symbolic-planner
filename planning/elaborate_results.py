import json
import os

import numpy as np

categories = [
    "Current_action",
    "Past_actions",
    "Future_actions",
]


def main(n_exp: int = 30) -> None:
    for category in categories:
        result_dir = f"results/{category}"

        gammas_list = []
        for i in range(n_exp):
            output_file = f"{result_dir}/experiment_{i+1}/output.json"

            with open(output_file, "r") as file:
                results = json.load(file)

            gammas = results["gamma"]
            gammas_list.append(gammas)

        with open(f"{result_dir}/results.txt", "w") as file:
            for gamma in gammas_list:
                file.write(f"{gamma}\n")

        gammas_average = np.mean(gammas_list)
        gammas_std = np.std(gammas_list)

        print(f"Category: {category}\n")
        print(f"Gamma statistics: {gammas_average:.2f} +- {gammas_std:.2f}\n")


if __name__ == "__main__":
    main()
