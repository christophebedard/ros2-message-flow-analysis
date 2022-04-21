#!/usr/bin/env python3
# Copyright 2022 Christophe Bedard
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Helper script to plot the results of the end-to-end latency overhead experiment."""

import sys
from typing import List
from typing import Tuple

import matplotlib.pyplot as plt

import numpy as np


# Whether to include titles above plots
include_plot_title = False


def get_default_colors() -> List[str]:
    """Get the list of default matplotlib colours."""
    return [p['color'] for p in plt.rcParams['axes.prop_cycle']]


def get_latencies(filename: str) -> np.array:
    """
    Load latency values from text file (1/line).

    Converts the latency values from ns to ms.

    :param filename: the file name
    :return: the latencies in milliseconds as an array
    """
    with open(filename) as f:
        lines = f.readlines()
        return np.genfromtxt(lines, dtype=int) / 1000000.0


def compute_theoretical_overhead(
) -> Tuple[float, float]:
    """
    Compute theoretical overhead based on overhead from:
    1. ros2_tracing paper (bedard2022ros2tracing)
    2. survey paper (gebai2018survey)
    and number of tracepoints in the hot path and number of message exchanges.

    :return: tuple with (theoretical overhead from ros2_tracing, theoretical overhead from survey)
    """
    # Single end-to-end latency overhead from ros2_tracing
    ros2_tracing_single_end_to_end_overhead = 0.0033
    # Number of tracepoints in the pub-sub hot path for ros2_tracing
    ros2_tracing_num_hotpath_tracepoints = 10
    # Single tracepoint overhead from survey (ns to ms)
    survey_overhead_per_tracepoint = 158.0 / 1000000.0
    # Number of tracepoints in the pub-sub hot path for message flow
    message_flow_num_hotpath_tracepoints = 13
    # Number of pub-sub exchanges in the overhead evaluation experiment
    message_flow_num_message_exchanges = 5

    from_ros2_tracing = message_flow_num_message_exchanges * (float(message_flow_num_hotpath_tracepoints) / float(ros2_tracing_num_hotpath_tracepoints)) * ros2_tracing_single_end_to_end_overhead
    from_survey = message_flow_num_message_exchanges * message_flow_num_hotpath_tracepoints * survey_overhead_per_tracepoint
    return from_ros2_tracing, from_survey


def plot(
    filename_notracing: str,
    filename_tracing: str,
    title: str = 'End-to-end latency comparison',
    ylabel: str = 'end-to-end latency (ms)',
    figure_filename: str = '7_end-to-end_latency_comparison',
) -> None:
    """
    Plot latencies.

    :param filename_notracing: the file name for no-tracing latencies
    :param filename_tracing: the file name for tracing latencies
    """
    latencies_notracing = get_latencies(filename_notracing)
    latencies_tracing = get_latencies(filename_tracing)
    print(latencies_notracing)
    print(latencies_tracing)

    base_mean = latencies_notracing.mean()
    base_stdev = latencies_notracing.std()
    base_median = np.median(latencies_notracing)
    base_q = np.percentile(latencies_notracing, [25, 75])
    trace_mean = latencies_tracing.mean()
    trace_stdev = latencies_tracing.std()
    trace_median = np.median(latencies_tracing)
    trace_q = np.percentile(latencies_tracing, [25, 75])
    th_overhead_ros2_tracing, th_overhead_survey = compute_theoretical_overhead()
    def format_num(num: float) -> str:
        fmt = ''
        if num < 100.0:
            fmt += ' '
        if num < 10.0:
            fmt += ' '
        return f'{fmt}{num:>3.10f}'
    print('End-to-end latencies: overhead')
    print('  base')
    print(f'    mean         = {format_num(base_mean)} ms')
    print(f'    stdev        = {format_num(base_stdev)} ms')
    print(f'    median       = {format_num(base_median)} ms')
    print(f'    Q1           = {format_num(base_q[0])} ms')
    print(f'    Q3           = {format_num(base_q[1])} ms')
    print('  trace')
    print(f'    mean         = {format_num(trace_mean)} ms')
    print(f'    stdev        = {format_num(trace_stdev)} ms')
    print(f'    median       = {format_num(trace_median)} ms')
    print(f'    Q1           = {format_num(trace_q[0])} ms')
    print(f'    Q3           = {format_num(trace_q[1])} ms')
    print('  difference')
    print(f'    mean         = {format_num(trace_mean - base_mean)} ms')
    print(f'    median       = {format_num(trace_median - base_median)} ms')
    print('  th. difference')
    print(f'    ros2_tracing = {format_num(th_overhead_ros2_tracing)} ms')
    print(f'    survey       = {format_num(th_overhead_survey)} ms')

    fig, ax = plt.subplots(1, 1, constrained_layout=True)

    colours = get_default_colors()
    boxcolourprop = {'color': colours[0], 'linewidth': 1.5}
    medianprops = {'color': colours[1], 'linewidth': 1.5}
    ax.boxplot(
        latencies_notracing, positions=[1],
        showfliers=False,
        medianprops=medianprops, boxprops=boxcolourprop, whiskerprops=boxcolourprop, capprops=boxcolourprop,
    )
    ax.boxplot(
        latencies_tracing, positions=[2],
        showfliers=False,
        medianprops=medianprops, boxprops=boxcolourprop, whiskerprops=boxcolourprop, capprops=boxcolourprop,
    )

    if include_plot_title:
        ax.set(title=title)

    ax.grid()
    ax.set_xticklabels(['without tracing', 'with tracing'])
    ax.set(ylabel=ylabel)

    filename = f'./{figure_filename}'
    fig.savefig(f'{filename}.png')
    fig.savefig(f'{filename}.svg')
    fig.savefig(f'{filename}.pdf')


def main(argv=sys.argv[1:]) -> int:
    """Plot."""
    if len(argv) != 2:
        print('error: must provide 2 arguments: path/to/latencies_no-tracing_*.txt path/to/latencies_tracing_*.txt')
        return 1
    filename_notracing = argv[0]
    filename_tracing = argv[1]
    if not filename_notracing.startswith('latencies_no-tracing_') or not filename_tracing.startswith('latencies_tracing_'):
        print('error: files seem to be inverted: expecting *no-tracing* then *tracing*')
        return 1

    plt.rcParams.update({
        'text.usetex': True,
        'font.family': 'serif',
        'font.size': 14,
        'axes.titlesize': 20,
    })

    plot(filename_notracing, filename_tracing)
    plt.show()

    return 0


if __name__ == '__main__':
    sys.exit(main())
