import io
import re

import matplotlib.pyplot as plt
import pandas as pd
from pandas.io.parsers.readers import csv

# 1. Define your log file path
file_path = "output_10.log"

# 2. Extract and parse data
cleaned_data = []

import re


def clean_and_extract_logs(log_lines):
    """
    Cleans log lines of ANSI colors, filters by DCONT_DBG,
    and ensures lines contain only numbers, commas, and dots.
    """
    ansi_escape = re.compile(r"\x1b\[[0-9;]*m")
    # This pattern matches strings containing only digits, commas, and dots
    # ^ = start of string, $ = end of string, + = at least one character
    allowed_pattern = re.compile(r"^[0-9,.\-]+$")

    csv_data = []

    for line in log_lines:
        clean_line = ansi_escape.sub("", line)

        if "DCONT_DBG:" in clean_line:
            parts = clean_line.split("DCONT_DBG:")
            if len(parts) > 1:
                row_values = parts[1].strip()

                # Only add to csv_data if it matches the strict pattern
                if allowed_pattern.match(row_values):
                    csv_data.append(row_values + "\n")

    return csv_data


with open(file_path, "r") as f:
    lines = f.readlines()
    clean = clean_and_extract_logs(lines)
    with open("out.csv", "w") as f:
        f.write(
            "millis,rot_x,rot_y,rot_z,angvel_x,angvel_y,angvel_z,target_mot_x,target_mot_y,target_mot_z,target_angular_vel_x,target_angular_vel_y,target_angular_vel_z,target_rot_x,target_rot_y,target_rot_z\n"
        )
        f.writelines(clean)

df = pd.read_csv("out.csv")

# Define groups based on your header
groups = {
    "x": ["target_rot_x", "rot_x", "target_angular_vel_x", "angvel_x", "target_mot_x"],
    "y": ["target_rot_y", "rot_y", "target_angular_vel_y", "angvel_y", "target_mot_y"],
    "z": ["target_rot_z", "rot_z", "target_angular_vel_z", "angvel_z", "target_mot_z"],
}

# Create subplots
fig, axes = plt.subplots(len(groups), 1, figsize=(12, 18), sharex=True)

# Plotting each group
for ax, (title, columns) in zip(axes, groups.items()):
    for col in columns:
        # ax.plot(df.index, df[col], label=col)
        ax.plot(df["millis"], df[col], label=col)
    ax.set_title(title)
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.7)

plt.xlabel("Time Index / Sample")
plt.tight_layout()
plt.show()
