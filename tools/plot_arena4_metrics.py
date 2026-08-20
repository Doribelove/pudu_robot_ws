#!/usr/bin/env python3
"""Create compact Arena4 evaluation plots from metrics.csv."""

import argparse
import ast
import csv
import os

import matplotlib.pyplot as plt


def parse_value(value):
    try:
        return ast.literal_eval(value)
    except (ValueError, SyntaxError):
        return value


def as_float_list(value):
    parsed = parse_value(value)
    if isinstance(parsed, (list, tuple)):
        return [float(item) for item in parsed]
    return []


def as_path(value):
    parsed = parse_value(value)
    if not isinstance(parsed, (list, tuple)):
        return []
    return [point for point in parsed
            if isinstance(point, (list, tuple)) and len(point) >= 2]


def load_metrics(path):
    with open(path, newline="") as stream:
        return list(csv.DictReader(stream))


def result_color(result):
    return {
        "GOAL_REACHED": "tab:green",
        "COLLISION": "tab:red",
        "TIMEOUT": "tab:orange",
    }.get(result, "tab:blue")


def plot_paths(rows, output):
    fig, axis = plt.subplots(figsize=(9, 7))
    for row in rows:
        path = as_path(row.get("path", "[]"))
        if not path:
            continue
        x = [point[0] for point in path]
        y = [point[1] for point in path]
        axis.plot(x, y, color=result_color(row.get("result", "")),
                  linewidth=1.0, alpha=0.8,
                  label=f"episode {row.get('episode')} ({row.get('result')})")
    axis.set_title("Arena4 paths")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.axis("equal")
    if rows and len(rows) <= 12:
        axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(output, "paths.png"), dpi=160)
    plt.close(fig)


def plot_episode_signals(rows, output):
    for row in rows:
        episode = row.get("episode", "unknown")
        time = as_float_list(row.get("time", "[]"))
        velocity = as_float_list(row.get("velocity", "[]"))
        angular = as_float_list(row.get("angular_velocity", "[]"))
        if not time:
            continue
        count = min(len(time), len(velocity), len(angular))
        fig, axis = plt.subplots(figsize=(10, 4.5))
        axis.plot(time[:count], velocity[:count], label="linear speed [m/s]")
        axis.plot(time[:count], angular[:count], label="angular speed [rad/s]")
        axis.set_title(f"Episode {episode}: velocity ({row.get('result', '')})")
        axis.set_xlabel("time [s]")
        axis.grid(alpha=0.25)
        axis.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(output, f"episode_{episode}_velocity.png"), dpi=160)
        plt.close(fig)


def write_summary(rows, output):
    fields = [
        "episode", "result", "path_length", "time_diff", "collision_amount",
        "max_velocity", "max_abs_angular_velocity",
    ]
    with open(os.path.join(output, "summary.csv"), "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            speeds = as_float_list(row.get("velocity", "[]"))
            angular = as_float_list(row.get("angular_velocity", "[]"))
            writer.writerow({
                "episode": row.get("episode", ""),
                "result": row.get("result", ""),
                "path_length": row.get("path_length", ""),
                "time_diff": row.get("time_diff", ""),
                "collision_amount": row.get("collision_amount", ""),
                "max_velocity": max(speeds, default=0.0),
                "max_abs_angular_velocity": max((abs(value) for value in angular), default=0.0),
            })


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", required=True, help="Directory containing metrics.csv")
    parser.add_argument("--out", default=None, help="Output directory for PNG/CSV files")
    args = parser.parse_args()

    data_dir = os.path.abspath(args.dir)
    output = os.path.abspath(args.out or os.path.join(data_dir, "plots"))
    os.makedirs(output, exist_ok=True)
    rows = load_metrics(os.path.join(data_dir, "metrics.csv"))
    if not rows:
        raise SystemExit("metrics.csv contains no evaluated episodes")
    plot_paths(rows, output)
    plot_episode_signals(rows, output)
    write_summary(rows, output)
    print(f"Wrote plots and summary to {output}")


if __name__ == "__main__":
    main()
