from __future__ import annotations

import argparse
import csv
import html
import math
import os
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "wpilog-current-analysis-mpl"))

import matplotlib
import numpy as np
import pandas as pd
from wpiutil.log import DataLogReader

matplotlib.use("Agg")
from matplotlib import pyplot as plt

ENABLED_KEY = "NT:/AdvantageKit/DriverStation/Enabled"
AUTONOMOUS_KEY = "NT:/AdvantageKit/DriverStation/Autonomous"
REAL_OUTPUTS_PREFIX = "NT:/AdvantageKit/RealOutputs/"
CURRENT_SUFFIXES = ("Supply Current", "Stator Current")
PHASES = ("enabled", "auto", "teleop")

MOTOR_GROUPS: dict[str, tuple[str, ...]] = {
    "Flywheel Left": ("Flywheel Left Lead", "Flywheel Left Follower"),
    "Flywheel Right": ("Flywheel Right Lead", "Flywheel Right Follower"),
    "Hood Left": ("Hood Left",),
    "Hood Right": ("Hood Right",),
    "Turret Left": ("Turret Left",),
    "Turret Right": ("Turret Right",),
    "Ground Pivot": ("Ground Pivot",),
    "Ground Rollers": ("Ground Roller Lead",),
    "Roller Floor": ("Roller Floor",),
    "B2": ("B2",),
    "Kicker": ("Kicker",),
}

DERIVED_GROUPS: dict[str, tuple[str, ...]] = {
    "Flywheels": ("Flywheel Left", "Flywheel Right"),
    "Hoods": ("Hood Left", "Hood Right"),
    "Turrets": ("Turret Left", "Turret Right"),
    "Intake": ("Ground Pivot", "Ground Rollers"),
    "Indexer": ("Roller Floor", "B2", "Kicker"),
    "Superstructure": tuple(MOTOR_GROUPS.keys()),
}

GLOBAL_CHANNELS: dict[str, str] = {
    "Battery Current": "NT:/AdvantageKit/SystemStats/BatteryCurrent",
    "Battery Voltage": "NT:/AdvantageKit/SystemStats/BatteryVoltage",
    "PDH Total Current": "NT:/AdvantageKit/PowerDistribution/TotalCurrent",
}

DIRECT_AGGREGATES: dict[str, dict[str, str]] = {
    "Drive": {
        "Supply Current": "NT:/AdvantageKit/RealOutputs/Drive/TotalDriveSupplyCurrent",
        "Stator Current": "NT:/AdvantageKit/RealOutputs/Drive/TotalDriveStatorCurrent",
        "Supply Current Abs": "NT:/AdvantageKit/RealOutputs/Drive/TotalDriveSupplyCurrentAbs",
        "Power": "NT:/AdvantageKit/RealOutputs/Drive/TotalDrivePower",
    }
}

CORE_MECHANISMS = list(MOTOR_GROUPS.keys())
DERIVED_SUMMARIES = list(DERIVED_GROUPS.keys()) + ["Drive"]


def current_topic(motor_name: str, current_type: str) -> str:
    return f"{REAL_OUTPUTS_PREFIX}{motor_name}/{current_type}"


def bucket_label(index: int, bucket_count: int) -> str:
    start = int(round(index * 100 / bucket_count))
    end = int(round((index + 1) * 100 / bucket_count))
    return f"{start}_{end}"


def bucket_column_names(bucket_count: int) -> list[str]:
    return [f"bucket_{bucket_label(index, bucket_count)}_avg" for index in range(bucket_count)]


@dataclass
class WeightedIntervalStats:
    intervals: list[tuple[float, float]] = field(default_factory=list)
    total_duration: float = 0.0
    weighted_sum: float = 0.0
    max_value: float = 0.0
    update_count: int = 0

    def add_interval(self, value: float, duration_seconds: float) -> None:
        if duration_seconds <= 0.0:
            return
        self.intervals.append((value, duration_seconds))
        self.total_duration += duration_seconds
        self.weighted_sum += value * duration_seconds
        self.max_value = max(self.max_value, value)

    def note_update(self) -> None:
        self.update_count += 1

    def average(self) -> float:
        if self.total_duration <= 0.0:
            return math.nan
        return self.weighted_sum / self.total_duration

    def weighted_percentile(self, percentile: float) -> float:
        if self.total_duration <= 0.0:
            return math.nan
        target = self.total_duration * percentile
        elapsed = 0.0
        sorted_intervals = sorted(self.intervals, key=lambda item: item[0])
        for value, duration in sorted_intervals:
            elapsed += duration
            if elapsed >= target:
                return value
        return sorted_intervals[-1][0]

    def bucket_averages(self, bucket_count: int) -> list[float]:
        if self.total_duration <= 0.0:
            return [math.nan] * bucket_count

        sorted_intervals = sorted(self.intervals, key=lambda item: item[0])
        bucket_width = self.total_duration / bucket_count
        bucket_weighted_sum = [0.0] * bucket_count
        bucket_duration = [0.0] * bucket_count
        bucket_index = 0
        bucket_remaining = bucket_width

        for value, duration in sorted_intervals:
            remaining = duration
            while remaining > 1e-12 and bucket_index < bucket_count:
                slice_duration = remaining if bucket_index == bucket_count - 1 else min(remaining, bucket_remaining)
                bucket_weighted_sum[bucket_index] += value * slice_duration
                bucket_duration[bucket_index] += slice_duration
                remaining -= slice_duration

                if bucket_index < bucket_count - 1:
                    bucket_remaining -= slice_duration
                    if bucket_remaining <= 1e-12:
                        bucket_index += 1
                        bucket_remaining = bucket_width

        return [
            bucket_weighted_sum[index] / bucket_duration[index] if bucket_duration[index] > 0.0 else math.nan
            for index in range(bucket_count)
        ]


@dataclass
class ChannelState:
    value: float = 0.0
    seen: bool = False


@dataclass
class PhaseWindow:
    stats: WeightedIntervalStats = field(default_factory=WeightedIntervalStats)
    active_since: int | None = None
    segments: list[tuple[int, int, float]] = field(default_factory=list)


@dataclass
class AggregateChannel:
    member_topics: tuple[str, ...]
    present_topics: set[str] = field(default_factory=set)
    current_total: float = 0.0
    phases: dict[str, PhaseWindow] = field(
        default_factory=lambda: {phase: PhaseWindow() for phase in PHASES}
    )


@dataclass
class AnalysisResult:
    rows: list[dict[str, str]]
    segments: dict[tuple[str, str, str], list[tuple[int, int, float]]]
    log_start_timestamp: int


def build_aggregate_specs() -> dict[str, tuple[str, ...]]:
    specs: dict[str, tuple[str, ...]] = {}

    for subsystem, motors in MOTOR_GROUPS.items():
        for current_type in CURRENT_SUFFIXES:
            specs[f"{subsystem}|{current_type}"] = tuple(current_topic(motor, current_type) for motor in motors)

    for subsystem, members in DERIVED_GROUPS.items():
        for current_type in CURRENT_SUFFIXES:
            topics: list[str] = []
            for member in members:
                topics.extend(current_topic(motor, current_type) for motor in MOTOR_GROUPS[member])
            specs[f"{subsystem}|{current_type}"] = tuple(topics)

    for subsystem, metrics in DIRECT_AGGREGATES.items():
        for metric, topic in metrics.items():
            specs[f"{subsystem}|{metric}"] = (topic,)

    return specs


def format_number(value: float) -> str:
    if math.isnan(value):
        return ""
    return f"{value:.3f}"


def discover_topics(log_path: Path) -> dict[int, tuple[str, str]]:
    entry_map: dict[int, tuple[str, str]] = {}
    direct_topics = {topic for metrics in DIRECT_AGGREGATES.values() for topic in metrics.values()}
    reader = DataLogReader(str(log_path))
    if not reader.isValid():
        raise ValueError(f"{log_path} is not a valid WPILOG file")

    for record in reader:
        if not record.isStart():
            continue
        start_data = record.getStartData()
        name = start_data.name
        data_type = start_data.type
        if name in {ENABLED_KEY, AUTONOMOUS_KEY} or name in GLOBAL_CHANNELS.values() or name in direct_topics:
            entry_map[start_data.entry] = (name, data_type)
            continue
        if name.startswith(REAL_OUTPUTS_PREFIX) and name.endswith(CURRENT_SUFFIXES):
            entry_map[start_data.entry] = (name, data_type)
    return entry_map


def phase_active(phase: str, enabled: bool, autonomous: bool) -> bool:
    if phase == "enabled":
        return enabled
    if phase == "auto":
        return enabled and autonomous
    if phase == "teleop":
        return enabled and not autonomous
    raise ValueError(f"Unknown phase {phase}")


def analyze_log(log_path: Path, bucket_count: int) -> AnalysisResult:
    entry_map = discover_topics(log_path)
    aggregate_specs = build_aggregate_specs()
    channel_states: dict[str, ChannelState] = {}
    channels_to_aggregates: dict[str, list[AggregateChannel]] = {}
    aggregates: dict[str, AggregateChannel] = {}

    for aggregate_name, topics in aggregate_specs.items():
        aggregate = AggregateChannel(member_topics=topics)
        aggregates[aggregate_name] = aggregate
        for topic in topics:
            channel_states.setdefault(topic, ChannelState())
            channels_to_aggregates.setdefault(topic, []).append(aggregate)

    for global_name, topic in GLOBAL_CHANNELS.items():
        aggregate = AggregateChannel(member_topics=(topic,))
        aggregates[f"{global_name}|Value"] = aggregate
        channel_states.setdefault(topic, ChannelState())
        channels_to_aggregates.setdefault(topic, []).append(aggregate)

    enabled = False
    autonomous = False
    enabled_seen = False
    autonomous_seen = False
    last_log_timestamp = 0
    first_relevant_timestamp: int | None = None

    def close_phase_window(window: PhaseWindow, timestamp: int, current_total: float) -> None:
        if window.active_since is None:
            return
        window.segments.append((window.active_since, timestamp, current_total))
        window.stats.add_interval(current_total, (timestamp - window.active_since) / 1_000_000.0)
        window.active_since = timestamp

    def apply_phase_transition(timestamp: int, old_enabled: bool, old_autonomous: bool) -> None:
        for aggregate in aggregates.values():
            for phase in PHASES:
                was_active = phase_active(phase, old_enabled, old_autonomous)
                is_active = phase_active(phase, enabled, autonomous)
                window = aggregate.phases[phase]
                if was_active and not is_active:
                    close_phase_window(window, timestamp, aggregate.current_total)
                    window.active_since = None
                elif not was_active and is_active:
                    window.active_since = timestamp

    for record in DataLogReader(str(log_path)):
        last_log_timestamp = record.getTimestamp()
        if record.isStart() or record.isFinish() or record.isSetMetadata():
            continue

        entry = record.getEntry()
        if entry not in entry_map:
            continue

        topic_name, data_type = entry_map[entry]
        timestamp = record.getTimestamp()
        if first_relevant_timestamp is None:
            first_relevant_timestamp = timestamp

        if topic_name == ENABLED_KEY:
            if data_type != "boolean":
                raise TypeError(f"Enabled topic {topic_name} had unexpected type {data_type}")
            old_enabled, old_autonomous = enabled, autonomous
            enabled = record.getBoolean()
            enabled_seen = True
            apply_phase_transition(timestamp, old_enabled, old_autonomous)
            continue

        if topic_name == AUTONOMOUS_KEY:
            if data_type != "boolean":
                raise TypeError(f"Autonomous topic {topic_name} had unexpected type {data_type}")
            old_enabled, old_autonomous = enabled, autonomous
            autonomous = record.getBoolean()
            autonomous_seen = True
            apply_phase_transition(timestamp, old_enabled, old_autonomous)
            continue

        if data_type == "double":
            value = record.getDouble()
        elif data_type == "float":
            value = record.getFloat()
        elif data_type in {"int", "int64"}:
            value = float(record.getInteger())
        else:
            continue

        state = channel_states[topic_name]
        previous_value = state.value
        state.value = value
        state.seen = True

        for aggregate in channels_to_aggregates[topic_name]:
            aggregate.present_topics.add(topic_name)
            for phase in PHASES:
                if phase_active(phase, enabled, autonomous):
                    close_phase_window(aggregate.phases[phase], timestamp, aggregate.current_total)
            aggregate.current_total += value - previous_value
            for phase in PHASES:
                if phase_active(phase, enabled, autonomous):
                    aggregate.phases[phase].active_since = timestamp
                    aggregate.phases[phase].stats.note_update()

    for aggregate in aggregates.values():
        for phase in PHASES:
            if phase_active(phase, enabled, autonomous):
                close_phase_window(aggregate.phases[phase], last_log_timestamp, aggregate.current_total)

    bucket_columns = bucket_column_names(bucket_count)
    rows: list[dict[str, str]] = []
    for aggregate_name, aggregate in sorted(aggregates.items()):
        subsystem, metric = aggregate_name.split("|", 1)
        has_data = bool(aggregate.present_topics)
        for phase in PHASES:
            stats = aggregate.phases[phase].stats
            buckets = stats.bucket_averages(bucket_count) if has_data else [math.nan] * bucket_count
            row = {
                "phase": phase,
                "subsystem": subsystem,
                "metric": metric,
                "enabled_state_seen": str(enabled_seen).lower(),
                "autonomous_state_seen": str(autonomous_seen).lower(),
                "phase_seconds": format_number(stats.total_duration),
                "member_topics_expected": str(len(aggregate.member_topics)),
                "member_topics_present": str(len(aggregate.present_topics)),
                "update_count_in_phase": str(stats.update_count),
                "average": format_number(stats.average() if has_data else math.nan),
                "p95": format_number(stats.weighted_percentile(0.95) if has_data else math.nan),
                "max": format_number(stats.max_value if has_data and stats.total_duration > 0.0 else math.nan),
                "top_bucket_avg": format_number(buckets[-1]),
                "bucket_count": str(bucket_count),
                "member_topics": "; ".join(aggregate.member_topics),
            }
            for column, value in zip(bucket_columns, buckets):
                row[column] = format_number(value)
            rows.append(row)
    segments: dict[tuple[str, str, str], list[tuple[int, int, float]]] = {}
    for aggregate_name, aggregate in aggregates.items():
        subsystem, metric = aggregate_name.split("|", 1)
        for phase in PHASES:
            segments[(phase, subsystem, metric)] = aggregate.phases[phase].segments

    return AnalysisResult(
        rows=rows,
        segments=segments,
        log_start_timestamp=first_relevant_timestamp or 0,
    )


def write_csv(rows: Iterable[dict[str, str]], output_path: Path) -> None:
    rows = list(rows)
    if not rows:
        raise ValueError("No rows produced")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def rows_to_dataframe(rows: list[dict[str, str]], bucket_count: int) -> pd.DataFrame:
    df = pd.DataFrame(rows).copy()
    numeric_cols = [
        "phase_seconds",
        "member_topics_expected",
        "member_topics_present",
        "update_count_in_phase",
        "average",
        "p95",
        "max",
        "top_bucket_avg",
        "bucket_count",
    ] + bucket_column_names(bucket_count)
    for column in numeric_cols:
        df[column] = pd.to_numeric(df[column], errors="coerce")
    df["coverage_ratio"] = df["member_topics_present"] / df["member_topics_expected"]
    return df


def barh_plot(df: pd.DataFrame, value_col: str, title: str, output_path: Path, limit: int = 12) -> None:
    plot_df = df.dropna(subset=[value_col]).sort_values(value_col, ascending=True).tail(limit)
    fig, ax = plt.subplots(figsize=(7.4, 4.2), constrained_layout=True)
    ax.barh(plot_df["subsystem"], plot_df[value_col])
    ax.set_title(title)
    ax.set_xlabel("Amps")
    ax.grid(axis="x", linestyle="--", alpha=0.4)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def bucket_heatmap(df: pd.DataFrame, bucket_count: int, title: str, output_path: Path) -> None:
    if df.empty:
        return
    bucket_cols = bucket_column_names(bucket_count)
    values = df[bucket_cols].to_numpy(dtype=float)
    masked = np.ma.masked_invalid(values)
    fig_height = max(3.3, len(df) * 0.32)
    fig, ax = plt.subplots(figsize=(7.2, fig_height), constrained_layout=True)
    image = ax.imshow(masked, aspect="auto")
    labels = [bucket_label(index, bucket_count).replace("_", "-") + "%" for index in range(bucket_count)]
    ax.set_xticks(range(bucket_count), labels)
    ax.set_yticks(range(len(df)), df["subsystem"].tolist())
    ax.set_title(title)
    fig.colorbar(image, ax=ax, label="Amps")
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def coverage_plot(df: pd.DataFrame, output_path: Path) -> None:
    plot_df = (
        df[["subsystem", "metric", "member_topics_expected", "member_topics_present", "coverage_ratio"]]
        .drop_duplicates()
        .sort_values("coverage_ratio", ascending=True)
    )
    labels = (plot_df["subsystem"] + " | " + plot_df["metric"]).tolist()
    fig_height = max(4.0, len(plot_df) * 0.18)
    fig, ax = plt.subplots(figsize=(7.8, fig_height), constrained_layout=True)
    ax.barh(labels, plot_df["coverage_ratio"] * 100.0)
    ax.set_title("Topic Coverage")
    ax.set_xlabel("Coverage (%)")
    ax.set_xlim(0, 105)
    ax.grid(axis="x", linestyle="--", alpha=0.4)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def build_step_series(
    segments: list[tuple[int, int, float]], origin_timestamp: int
) -> tuple[list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []
    for start, end, value in segments:
        xs.extend([(start - origin_timestamp) / 1_000_000.0, (end - origin_timestamp) / 1_000_000.0, math.nan])
        ys.extend([value, value, math.nan])
    return xs, ys


def time_series_plot(
    df: pd.DataFrame,
    segments: dict[tuple[str, str, str], list[tuple[int, int, float]]],
    origin_timestamp: int,
    phase: str,
    metric: str,
    output_path: Path,
    limit: int = 5,
) -> None:
    plot_df = df[
        (df["phase"] == phase) & (df["metric"] == metric) & (df["subsystem"].isin(CORE_MECHANISMS))
    ].dropna(subset=["top_bucket_avg"]).sort_values("top_bucket_avg", ascending=False).head(limit)

    fig, ax = plt.subplots(figsize=(8.3, 4.6), constrained_layout=True)
    for _, row in plot_df.iterrows():
        key = (phase, row["subsystem"], metric)
        xs, ys = build_step_series(segments.get(key, []), origin_timestamp)
        if xs:
            ax.plot(xs, ys, linewidth=1.0, label=row["subsystem"])
    ax.set_title(f"{phase.title()} {metric} Top 5 Time Series")
    ax.set_xlabel("Log Time (s)")
    ax.set_ylabel("Amps")
    ax.grid(True, linestyle="--", alpha=0.35)
    if not plot_df.empty:
        ax.legend(fontsize=8, ncols=2)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def build_report_assets(
    result: AnalysisResult, report_dir: Path, bucket_count: int
) -> tuple[pd.DataFrame, list[dict[str, str]]]:
    report_dir.mkdir(parents=True, exist_ok=True)
    df = rows_to_dataframe(result.rows, bucket_count)
    currents = df[df["metric"].isin(CURRENT_SUFFIXES)].copy()
    sections: list[dict[str, str]] = []

    for phase in PHASES:
        for metric in CURRENT_SUFFIXES:
            phase_metric = currents[
                (currents["phase"] == phase)
                & (currents["metric"] == metric)
                & (currents["subsystem"].isin(CORE_MECHANISMS))
            ].copy()

            average_filename = f"{phase}_{metric.lower().replace(' ', '_')}_average.png"
            top_filename = f"{phase}_{metric.lower().replace(' ', '_')}_top_bucket.png"
            heatmap_filename = f"{phase}_{metric.lower().replace(' ', '_')}_heatmap.png"
            series_filename = f"{phase}_{metric.lower().replace(' ', '_')}_timeseries.png"

            barh_plot(
                phase_metric,
                "average",
                f"{phase.title()} {metric} Average by Mechanism",
                report_dir / average_filename,
            )
            barh_plot(
                phase_metric,
                "top_bucket_avg",
                f"{phase.title()} {metric} Highest Bucket by Mechanism",
                report_dir / top_filename,
            )
            bucket_heatmap(
                phase_metric.sort_values("top_bucket_avg", ascending=False),
                bucket_count,
                f"{phase.title()} {metric} Buckets",
                report_dir / heatmap_filename,
            )
            time_series_plot(
                currents,
                result.segments,
                result.log_start_timestamp,
                phase,
                metric,
                report_dir / series_filename,
            )

            sections.append(
                {
                    "phase": phase,
                    "metric": metric,
                    "average": average_filename,
                    "top_bucket": top_filename,
                    "heatmap": heatmap_filename,
                    "timeseries": series_filename,
                }
            )

    coverage_plot(df, report_dir / "coverage.png")
    return df, sections


def dataframe_table_html(df: pd.DataFrame, columns: list[str]) -> str:
    header = "".join(f"<th>{html.escape(column)}</th>" for column in columns)
    body_rows: list[str] = []
    for _, row in df.iterrows():
        cells: list[str] = []
        for column in columns:
            value = row[column]
            if isinstance(value, float):
                text = "" if math.isnan(value) else f"{value:.3f}"
            else:
                text = str(value)
            cells.append(f"<td>{html.escape(text)}</td>")
        body_rows.append("<tr>" + "".join(cells) + "</tr>")
    return f"<table border='1' cellspacing='0' cellpadding='4'><thead><tr>{header}</tr></thead><tbody>{''.join(body_rows)}</tbody></table>"


def write_html_report(
    df: pd.DataFrame, report_dir: Path, log_path: Path, csv_path: Path, sections: list[dict[str, str]], bucket_count: int
) -> Path:
    bucket_cols = bucket_column_names(bucket_count)
    top_rows = (
        df[
            (df["metric"].isin(CURRENT_SUFFIXES))
            & (df["subsystem"].isin(CORE_MECHANISMS))
        ]
        .sort_values("top_bucket_avg", ascending=False)
        .head(12)[["phase", "subsystem", "metric", "average", "top_bucket_avg", "max"]]
    )
    missing_rows = (
        df[df["member_topics_present"] < df["member_topics_expected"]][
            ["phase", "subsystem", "metric", "member_topics_present", "member_topics_expected"]
        ]
        .drop_duplicates()
    )
    supply_rows = df[df["metric"] == "Supply Current"].copy().sort_values(
        ["average", "phase", "subsystem"], ascending=[False, True, True]
    )
    stator_rows = df[df["metric"] == "Stator Current"].copy().sort_values(
        ["average", "phase", "subsystem"], ascending=[False, True, True]
    )
    other_rows = df[~df["metric"].isin(CURRENT_SUFFIXES)].copy().sort_values(
        ["average", "phase", "subsystem"], ascending=[False, True, True]
    )

    parts = [
        "<!DOCTYPE html>",
        "<html lang='en'>",
        "<head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>WPILOG Current Analysis Report</title></head>",
        "<body>",
        "<h1>WPILOG Current Analysis Report</h1>",
        f"<p><b>Log:</b> {html.escape(log_path.name)}</p>",
        f"<p><b>CSV:</b> {html.escape(csv_path.name)}</p>",
        f"<p><b>Configured buckets:</b> {bucket_count}</p>",
        "<p>This report is intentionally plain. Use the selectors below to switch between enabled/auto/teleop and supply/stator views.</p>",
        "<p>",
        "<label for='phaseSelect'><b>Phase:</b></label>",
        "<select id='phaseSelect'>"
        + "".join(f"<option value='{phase}'>{phase}</option>" for phase in PHASES)
        + "</select>",
        "&nbsp;&nbsp;",
        "<label for='metricSelect'><b>Metric:</b></label>",
        "<select id='metricSelect'>"
        + "".join(f"<option value='{metric}'>{metric}</option>" for metric in CURRENT_SUFFIXES)
        + "</select>",
        "</p>",
    ]

    for section in sections:
        parts.extend(
            [
                f"<div data-phase='{html.escape(section['phase'])}' data-metric='{html.escape(section['metric'])}' class='toggle-section'>",
                f"<h2>{html.escape(section['phase'].title())} {html.escape(section['metric'])}</h2>",
                "<p>Average ranking.</p>",
                f"<p><img src='{html.escape(section['average'])}' alt='Average ranking' width='820'></p>",
                "<p>Highest bucket ranking.</p>",
                f"<p><img src='{html.escape(section['top_bucket'])}' alt='Top bucket ranking' width='820'></p>",
                "<p>Bucket heatmap.</p>",
                f"<p><img src='{html.escape(section['heatmap'])}' alt='Bucket heatmap' width='820'></p>",
                "<p>Top 5 mechanism currents over time.</p>",
                f"<p><img src='{html.escape(section['timeseries'])}' alt='Time series' width='820'></p>",
                "</div>",
            ]
        )

    parts.extend(
        [
            "<h2>Coverage</h2>",
            "<p>Expected signals that were absent from the log.</p>",
            "<p><img src='coverage.png' alt='Topic coverage' width='820'></p>",
            "<h2>Top Enabled Rows</h2>",
            "<p>Highest top-bucket rows among direct mechanisms across all phases.</p>",
            dataframe_table_html(top_rows, ["phase", "subsystem", "metric", "average", "top_bucket_avg", "max"]),
            "<h2>Supply Rows</h2>",
            "<p>All supply-current CSV rows.</p>",
            dataframe_table_html(
                supply_rows,
                ["phase", "subsystem", "metric", "phase_seconds", "average", "p95", "max", "top_bucket_avg"]
                + bucket_cols,
            ),
            "<h2>Stator Rows</h2>",
            "<p>All stator-current CSV rows.</p>",
            dataframe_table_html(
                stator_rows,
                ["phase", "subsystem", "metric", "phase_seconds", "average", "p95", "max", "top_bucket_avg"]
                + bucket_cols,
            ),
            "<h2>Other Rows</h2>",
            "<p>Non-supply/stator CSV rows.</p>",
            dataframe_table_html(
                other_rows,
                ["phase", "subsystem", "metric", "phase_seconds", "average", "p95", "max", "top_bucket_avg"]
                + bucket_cols,
            ),
            "<h2>Missing Coverage Rows</h2>",
            dataframe_table_html(
                missing_rows, ["phase", "subsystem", "metric", "member_topics_present", "member_topics_expected"]
            ),
            """
<script>
function updateSections() {
  const phase = document.getElementById('phaseSelect').value;
  const metric = document.getElementById('metricSelect').value;
  document.querySelectorAll('.toggle-section').forEach((section) => {
    const matches = section.dataset.phase === phase && section.dataset.metric === metric;
    section.hidden = !matches;
  });
}
document.getElementById('phaseSelect').addEventListener('change', updateSections);
document.getElementById('metricSelect').addEventListener('change', updateSections);
updateSections();
</script>
""",
            "</body></html>",
        ]
    )

    html_path = report_dir / "index.html"
    html_path.write_text("\n".join(parts), encoding="utf-8")
    return html_path


def generate_report(result: AnalysisResult, report_dir: Path, log_path: Path, csv_path: Path, bucket_count: int) -> Path:
    df, sections = build_report_assets(result, report_dir, bucket_count)
    return write_html_report(df, report_dir, log_path, csv_path, sections, bucket_count)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Summarize enabled-period subsystem current draw from an AdvantageKit/WPILOG file, "
            "split by enabled/auto/teleop, and optionally generate chart images plus a simple HTML report."
        )
    )
    parser.add_argument("log_path", type=Path, help="Path to the input .wpilog file")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Path to the output CSV. Defaults to <log_name>_current_summary.csv",
    )
    parser.add_argument(
        "--report-dir",
        type=Path,
        help="Directory for PNG charts and the HTML report. Defaults to <csv_stem>_report",
    )
    parser.add_argument(
        "--no-report",
        action="store_true",
        help="Skip generating PNG/HTML visualizations and write only the CSV.",
    )
    parser.add_argument(
        "--quantiles",
        type=int,
        default=4,
        help="How many current buckets to split enabled/auto/teleop time into. Default: 4.",
    )
    args = parser.parse_args()
    if args.quantiles < 2:
        parser.error("--quantiles must be at least 2")
    return args


def main() -> None:
    args = parse_args()
    log_path = args.log_path.resolve()
    output_path = args.output.resolve() if args.output else log_path.with_name(f"{log_path.stem}_current_summary.csv")
    report_dir = args.report_dir.resolve() if args.report_dir else output_path.with_name(f"{output_path.stem}_report")

    result = analyze_log(log_path, args.quantiles)
    write_csv(result.rows, output_path)
    print(output_path)

    if not args.no_report:
        html_path = generate_report(result, report_dir, log_path, output_path, args.quantiles)
        print(html_path)


if __name__ == "__main__":
    main()
