import re


def simplify_file(input_file, output_file):
    with open(input_file, "r") as f:
        lines = f.readlines()
        
    simplified_lines = []
    for line in lines:
        # Remove "_DETDUP_x" using regular expression
        line = re.sub(r"_DETDUP_\d+", "", line)

        # Remove (trans-x) actions
        line = re.sub(r"\(trans-\d+\s+l\d+\)", "", line)

        # Remove extra whitespace
        line = line.strip()

        # Append to the list of simplified lines
        if line:
            simplified_lines.append(line)
        
    with open(output_file, "w") as f:
        f.write("\n".join(simplified_lines))


def cut_lines_after_keyword(input_file, output_file):
    keyword = "Policy:"
    with open(input_file, "r") as f:
        lines = f.readlines()

    found_keyword = False
    output_lines = []

    for line in lines:
        if found_keyword:
            output_lines.append(line)
        elif keyword in line:
            found_keyword = True
            output_lines.append(line)

    with open(output_file, "w") as f:
        f.writelines(output_lines)


# Example usage:
input_file = "config/PDDL/sas_plan"
output_file = "config/PDDL/sas_plan_adapted"
simplify_file(input_file, output_file)

input_file = "config/PDDL/human_policy.pol"
output_file = "config/PDDL/human_policy.pol"
cut_lines_after_keyword(input_file, output_file)
