import json
import os

import numpy as np
import tyro


categories = ["Current_action", "Past_actions", "Future_actions"]


def main(n_exp: int = 15) -> None:
    for category in categories:
        result_dir = f"results/{category}"

        hits_list = []
        for i in range(n_exp):
            output_file = f"{result_dir}/experiment_{i+1}/output.json"

            with open(output_file, "r") as file:
                results = json.load(file)

            hits = results["hits"]
            hits_list.append(hits)

        with open(f"{result_dir}/results.txt", "w") as file:
            for hit in hits_list:
                file.write(f"{hit}\n")

        hits_average = np.mean(hits_list)
        hits_std = np.std(hits_list)

        print(f"Category: {category}\n")
        print(f"Hits percentage: {hits_average:.2f} +- {hits_std:.2f}\n")


if __name__ == "__main__":
    tyro.cli(main)
