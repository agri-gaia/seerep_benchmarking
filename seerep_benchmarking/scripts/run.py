#!/usr/bin/env python3

import json
import math
import os
import re
import shutil
import subprocess
import sys
from argparse import ArgumentParser
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import List, Tuple, Union

import matplotlib.pyplot as plt
import pandas as pd
import rospkg

# utility functions


def bytes_to_string(num_bytes: int) -> str:
    """
    Convert a number of bytes to a human-readable string.

    Args:
        num_bytes (int): Number of bytes.

    Returns:
        str: String representation of the number of bytes.

    """
    if num_bytes == 0:
        return "0B"
    units = ("B", "KiB", "MiB", "GiB", "TiB")
    unit_index = int(math.floor(math.log(num_bytes, 1024)))
    bytes_in_unit = int(round(num_bytes / math.pow(1024, unit_index)))
    return f"{bytes_in_unit}{units[unit_index]}"


def string_to_bytes(byte_str: str) -> int:
    """
    Converts a string representation of bytes to an integer value of bytes.

    Args:
        byte_str (str): String representation of bytes, e.g., "10MiB".

    Returns:
        int: Integer value of bytes.

    Examples:
        >>> string_to_bytes("10MiB")
        10485760
        >>> string_to_bytes("1GiB")
        1073741824
    """
    UNIT_MAP = {
        "B": 1,
        "KiB": 1024,
        "MiB": 1024**2,
        "GiB": 1024**3,
        "TiB": 1024**4,
    }
    split_result = re.split(r"(\d+\.\d+|\d+)", byte_str)
    return int(split_result[1]) * UNIT_MAP[split_result[2]]


def remove_files(directory: Path, extension: str) -> None:
    """
    Remove files with a specific extension from a directory.

    Args:
        directory (Path): Directory path.
        extension (str): File extension to be removed.

    Returns:
        None
    """
    for file in Path.glob(directory, "*" + extension):
        try:
            os.remove(file)
        except OSError as e:
            print(f"Error while deleting {file}: {e}")


def executable_path(
    package_name: str, executable_name: str
) -> Union[Path, None]:
    """
    Get the executable path for a given ROS package and executable name.

    Args:
        package (str): Name of the ROS package.
        name (str): Name of the executable.

    Returns:
        Union[Path, None]: The path to the executable if found, None otherwise.
    """
    try:
        subprocess_output = subprocess.run(
            ["catkin_find", package_name, executable_name],
            check=True,
            stdout=subprocess.PIPE,
        )
    except subprocess.CalledProcessError as e:
        print(
            f"Error while searching the executable {executable_name} in package {package_name}: {e}"
        )
        return None
    return Path(subprocess_output.stdout.decode("utf-8").strip())


# main benchmarking components
class SeerepBenchmarking:
    def __init__(self, config_path: str):
        self.usr_config = json.loads(open(config_path, "r").read())
        self.package_name = "seerep_benchmarking"
        self.executable_name = "seerep_benchmarking"
        self.subprocess_output = False
        self.data_dirs = [
            Path(self.usr_config["host_dir"] + "/" + directory)
            for directory in ["mcap", "hdf5"]
        ]

    def run(self) -> None:
        """Run the benchmark for with the provided user configuration."""
        payload_path = Path(
            f"{self.usr_config['host_dir']}/{self.usr_config['payload_file_name']}"
        )
        benchmark_path = executable_path(
            self.package_name, self.executable_name
        )

        if benchmark_path == Path("."):
            print("Could not find benchmark executable, source the workspace?")
            sys.exit(1)

        for total_size in self.usr_config["total_sizes"]:
            for message_size in self.usr_config["message_sizes"]:
                label = f"{bytes_to_string(message_size)}-{bytes_to_string(total_size)}"
                print(f"Running {label} ...")
                for run in range(self.usr_config["num_runs"]):
                    print(f"Run {run + 1} ...")
                    try:
                        # setup the required output directories
                        for directory in self.data_dirs:
                            if not directory.exists():
                                directory.mkdir()
                        output = subprocess.run(
                            [
                                benchmark_path,
                                str(payload_path),
                                label,
                                str(self.usr_config["host_dir"]),
                                str(message_size),
                                str(total_size),
                            ],
                            check=True,
                            stdout=subprocess.PIPE,
                        )
                        for directory in self.data_dirs:
                            shutil.rmtree(directory)
                    except subprocess.CalledProcessError as e:
                        print(f"Subprocess error while running benchmark: {e}")
                        sys.exit(1)
                    if self.subprocess_output:
                        print(f"{output.stdout.decode('utf-8')}")


# data processing functions and visualization


def read_csvs(csv_path: Path) -> List[pd.DataFrame]:
    """
    Combine the contents of all csv files in the path with the same total size.

    For example, if the path contains the following files:
    - 10MiB-100MiB.csv
    - 10MiB-1000MiB.csv
    - 100MiB-200MiB.csv

    The function will return a list of two pandas DataFrames, where the first
    DataFrame contains the contents of the first two files and the second
    DataFrame contains the contents of the last file.

    Args:
        csv_path (Path): Path to the directory containing the CSV files.

    Returns:
        List[pd.DataFrame]: A list of pandas DataFrames
    """
    if type(csv_path) is not Path:
        csv_path = Path(csv_path)

    # group the csv files by the total size
    d = defaultdict(list)
    for csv_file in Path.glob(csv_path, "*.csv"):
        if len(split_name := csv_file.stem.split("-")) > 1:
            d[split_name[1]].append(csv_file)

    # combine the csv files for each total size
    dfs = []
    for key in d.keys():
        df = pd.concat(
            [pd.read_csv(val).assign(label=val.stem) for val in d[key]]
        )
        df.attrs["title"] = key
        dfs.append(df)
    return dfs


def process_data(df: pd.DataFrame) -> pd.DataFrame:
    """
    Process the given DataFrame to calculate the mean and standard deviation.

    Args:
        df (pd.DataFrame): DataFrame to process.
    Returns:
        pd.DataFrame: A DataFrame containing the mean and standard deviation of
        the "gb/s" column, grouped by "msg_size", "label" and "file_type".
    """
    df["write_ns"] = df["write_ns"].apply(lambda x: x / 1e9)
    df.rename(columns={"write_ns": "write_s"}, inplace=True)

    df["written_bytes"] = df["written_bytes"].apply(lambda x: x / 1024**3)
    df.rename(columns={"written_bytes": "written_gb"}, inplace=True)

    df["msg_size"] = df["label"].apply(lambda x: x.split("-")[0])

    df["gb/s"] = df[["written_gb", "write_s"]].apply(
        lambda x: x.written_gb / x.write_s, axis=1
    )

    df = df.groupby(["label", "msg_size", "file_type"], as_index=False)[
        "gb/s"
    ].agg(["mean", "std"])

    df.reset_index(inplace=True)
    df.drop("label", axis=1, inplace=True)

    mean_df = df.pivot(index="msg_size", columns="file_type", values="mean")
    std_df = df.pivot(index="msg_size", columns="file_type", values="std")

    # sort by ascending msg sie
    mean_df.sort_values(
        by="msg_size", inplace=True, key=lambda x: x.apply(string_to_bytes)
    )
    std_df.sort_values(
        by="msg_size", inplace=True, key=lambda x: x.apply(string_to_bytes)
    )

    return (mean_df, std_df)


def set_size(width: float, fraction: float = 1) -> Tuple[float, float]:
    """
    Set figure dimensions to avoid scaling in LaTeX.

    Parameters:
        width (float): Width of the figure in points.
        fraction (float, optional): Fraction of the width to be used. Defaults
        to 1.

    Returns:
        Tuple[float, float]: The dimensions of the figure as a tuple of
        (width, height) in inches.
    """

    # Width of figure (in pts)
    fig_width_pt = width * fraction

    # Convert from pt to inches
    inches_per_pt = 1 / 72.27

    # Golden ratio to set aesthetic figure height
    # https://disq.us/p/2940ij3
    golden_ratio = (5**0.5 - 1) / 2

    # Figure width in inches
    fig_width_in = fig_width_pt * inches_per_pt
    # Figure height in inches
    fig_height_in = fig_width_in * golden_ratio
    fig_dim = (fig_width_in, fig_height_in)

    return fig_dim


def plot_data(
    mean_df: pd.DataFrame,
    df2: pd.DataFrame,
    title: str,
    output_dir: str,
    save_img: bool = True,
) -> None:
    """Plot or save the processed data in a bar chart.

    Args:
        mean_df (pd.DataFrame): Mean data to be plotted.
        df2 (pd.DataFrame): Error data to be plotted as error bars.
        title (str): Title of the plot (only used for the filename).
        output_dir (str): The directory where the plot should be saved.
        save_img (bool, optional): Whether to save the plot as an image.
        Defaults to True.

    Returns:
        None
    """
    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "serif",
            "axes.labelsize": 11,
            "axes.titlesize": 10,
            "font.size": 11,
            "legend.fontsize": 6,
            "legend.title_fontsize": 7,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
        }
    )

    ax = mean_df.plot(
        kind="bar",
        width=0.4,
        yerr=df2,
        color=["#a6a6a6", "#548235"],
        rot=45,
        capsize=2,
        edgecolor="white",
        linewidth=1,
        figsize=set_size(245.72, 1.3),
    )

    plt.xlabel(
        "Message Size",
        labelpad=10,
    )
    plt.ylabel(
        "Troughput in GiB/s",
    )
    plt.xticks()
    plt.yticks()
    ax = plt.gca()
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(axis="y", linestyle="dashed", linewidth=0.5, alpha=0.65)
    ax.set_axisbelow(True)
    ax.set_ylim([0, 3.5])
    ax.legend(title="File Format", loc="upper left")
    plt.tight_layout()

    if save_img:
        plt.savefig(
            Path(
                f"{output_dir}/{title}_{datetime.now().strftime('%Y_%m_%d_%H_%M_%S')}.pdf"
            ),
            format="pdf",
            bbox_inches="tight",
        )
    else:
        plt.show()


def main():
    parser = ArgumentParser()
    parser.add_argument(
        "--plot-only", action="store_true", help="Only plot previous data"
    )
    args = parser.parse_args()

    r = rospkg.RosPack()
    package_path = r.get_path("seerep_benchmarking")
    benchmark = SeerepBenchmarking(f"{package_path}/config/config.json")

    if not args.plot_only:
        benchmark.run()

    print("Reading csv files ...")
    csv_dfS = read_csvs(benchmark.usr_config["host_dir"])

    if not csv_dfS:
        print("No csv files found, exiting ...")
        sys.exit(1)

    print("Processing ...")
    for i, df in enumerate(csv_dfS):
        df1, df2 = process_data(df)
        print(f"Generating plot {i + 1}")
        plot_data(
            df1,
            df2,
            df.attrs["title"],
            benchmark.usr_config["host_dir"],
            save_img=True,
        )


if __name__ == "__main__":
    main()
