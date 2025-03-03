import numpy as np
import matplotlib.pyplot as plt
import json

# Increase fonts and other style parameters
plt.rcParams.update({
    "figure.autolayout": True,
    "axes.edgecolor": "0.3",
    "axes.linewidth": 1.2,
    "grid.linestyle": "--",
    "grid.alpha": 0.6,
    "font.family": "sans-serif",
    "font.size": 14,        # Base font size
    "axes.titlesize": 16,   # Title font size
    "axes.labelsize": 16,   # Axis label font size
    "xtick.labelsize": 14,  # X-tick label font size
    "ytick.labelsize": 14,  # Y-tick label font size
    "legend.fontsize": 14   # Legend font size
})

cases = ["Past_actions", "Future_actions"]
fig, axes = plt.subplots(1, 2, figsize=(14, 5))
plt.style.use("seaborn-v0_8-whitegrid")

for idx, case in enumerate(cases):
    datasets = []
    for i in range(30):
        with open(f"results/{case}/experiment_{i+1}/output.json", "r") as file:
            results = json.load(file)
            datasets.append(results["histogram"])

    max_length = max(len(d["union"]) for d in datasets)

    aligned_correct, aligned_missing, aligned_hallucinations = [], [], []

    for d in datasets:
        total = np.array(d["union"], dtype=float)
        correct = np.array(d["correct"], dtype=float) / total
        missing = np.array(d["missing"], dtype=float) / total
        hallucinations = np.array(d["hallucinations"], dtype=float) / total

        n = len(total)
        zero = d["zero"]  # 1-based index for reference

        if case == "Past_actions":
            # No reversal, last element aligns at x=0
            pad_left = max_length - n
            c_padded = np.pad(correct, (pad_left, 0), constant_values=np.nan)
            m_padded = np.pad(missing, (pad_left, 0), constant_values=np.nan)
            h_padded = np.pad(hallucinations, (pad_left, 0), constant_values=np.nan)

            aligned_correct.append(c_padded)
            aligned_missing.append(m_padded)
            aligned_hallucinations.append(h_padded)

        else:  # Future_actions
            # First element aligns at x=0, pad right
            pad_right = max_length - n
            c_padded = np.pad(correct, (0, pad_right), constant_values=np.nan)
            m_padded = np.pad(missing, (0, pad_right), constant_values=np.nan)
            h_padded = np.pad(hallucinations, (0, pad_right), constant_values=np.nan)

            aligned_correct.append(c_padded)
            aligned_missing.append(m_padded)
            aligned_hallucinations.append(h_padded)

    mean_correct = np.nanmean(np.array(aligned_correct), axis=0)
    mean_missing = np.nanmean(np.array(aligned_missing), axis=0)
    mean_hallucinations = np.nanmean(np.array(aligned_hallucinations), axis=0)

    # x-axis for Past_actions: [-max_length+1 ... 0]
    # x-axis for Future_actions: [0 ... max_length-1]
    if case == "Past_actions":
        x_axis = np.arange(-max_length + 1, 1)
    else:
        x_axis = np.arange(0, max_length)

    ax = axes[idx]
    colors = ["#4CAF50", "#2196F3", "#F44336"]  # Green, Blue, Red

    ax.bar(x_axis, mean_correct, color=colors[0], edgecolor='black', label="Correct fluents")
    ax.bar(x_axis, mean_missing, bottom=mean_correct, color=colors[1], edgecolor='black', label="Missing fluents")
    ax.bar(x_axis, mean_hallucinations, bottom=mean_correct + mean_missing,
           color=colors[2], edgecolor='black', label="Hallucinated fluents")

    # Show grid on y-axis only
    ax.grid(axis='y')

    # Show all x ticks
    ax.set_xticks(x_axis)
    ax.set_xticklabels(x_axis, rotation=0)

    # Vertical line at x=0
    ax.axvline(0, color='black', linestyle='--', linewidth=1.5, label="Present moment")

    ax.set_xlabel("Temporal timesteps", fontweight="bold")
    ax.set_ylabel("Ratio", fontweight="bold")
    ax.set_title(f"Partition of fluents in {case}", fontweight="bold")

    # Only add legend for Future_actions, in the lower right
    if case == "Future_actions":
        ax.legend(loc="lower right", frameon=True, edgecolor='black')

# Adjust layout
plt.tight_layout()

# Save the figure with high quality (dpi=300)
plt.savefig("Figure_1.png", dpi=300, bbox_inches='tight')

# Finally, show the figure
plt.show()
