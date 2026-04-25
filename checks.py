from __future__ import annotations
 
from dataclasses import dataclass, field
from wpiutil.log import DataLogReader
from pathlib import Path
 
BROWNOUT_KEY = "NT:/AdvantageKit/SystemStats/BrownedOut"
BATTERY_VOLTAGE_KEY = "NT:/AdvantageKit/SystemStats/BatteryVoltage"
ENABLED_KEY = "NT:/AdvantageKit/DriverStation/Enabled"
 
INDEXER_VELOCITY_TOPICS = {
    "Roller Floor": "NT:/AdvantageKit/RealOutputs/Roller Floor/Velocity",
    "B2": "NT:/AdvantageKit/RealOutputs/B2/Velocity",
    "Kicker": "NT:/AdvantageKit/RealOutputs/Kicker/Velocity",
}
 
FLYWHEEL_VELOCITY_TOPICS = {
    "Flywheel Left": "NT:/AdvantageKit/RealOutputs/Flywheel Left Lead/Velocity",
    "Flywheel Right": "NT:/AdvantageKit/RealOutputs/Flywheel Right Lead/Velocity",
}
 
# Thresholds
BROWNOUT_VOLTAGE_THRESHOLD = 7.0  # volts — flag if battery dips below this
FLICKER_MIN_VELOCITY = 1.0  # rps — ignore near-zero velocity noise
FLICKER_MIN_REVERSALS = 3  # number of direction changes to count as flickering
FLICKER_WINDOW_SECONDS = 2.0  # time window to look for reversals in
FLYWHEEL_VELOCITY_MIN = 10.0  # rps — only check direction if spinning fast enough
 
 
@dataclass
class CheckResult:
    name: str
    passed: bool
    message: str
 
 
@dataclass
class CheckReport:
    results: list[CheckResult] = field(default_factory=list)
 
    def add(self, result: CheckResult) -> None:
        self.results.append(result)
 
    def print_summary(self) -> None:
        print("\n=== Automated Checks ===")
        for result in self.results:
            status = "PASS" if result.passed else "FAIL"
            print(f"[{status}] {result.name}: {result.message}")
        print()
 
 
def run_checks(log_path: Path) -> CheckReport:
    report = CheckReport()
 
    # Discover which entry IDs map to which topics
    all_topics = {
        BROWNOUT_KEY,
        BATTERY_VOLTAGE_KEY,
        ENABLED_KEY,
        *INDEXER_VELOCITY_TOPICS.values(),
        *FLYWHEEL_VELOCITY_TOPICS.values(),
    }
    entry_map: dict[int, str] = {}
    reader = DataLogReader(str(log_path))
    for record in reader:
        if record.isStart():
            d = record.getStartData()
            if d.name in all_topics:
                entry_map[d.entry] = d.name
 
    # Storage for time-series data
    brownout_events: list[float] = []
    battery_voltage_dips: list[tuple[float, float]] = []  # (timestamp_s, voltage)
    # indexer: motor_name -> list of (timestamp_s, velocity)
    indexer_series: dict[str, list[tuple[float, float]]] = {k: [] for k in INDEXER_VELOCITY_TOPICS}
    flywheel_series: dict[str, list[tuple[float, float]]] = {k: [] for k in FLYWHEEL_VELOCITY_TOPICS}
    enabled = False
 
    topic_to_indexer = {v: k for k, v in INDEXER_VELOCITY_TOPICS.items()}
    topic_to_flywheel = {v: k for k, v in FLYWHEEL_VELOCITY_TOPICS.items()}
 
    for record in DataLogReader(str(log_path)):
        if record.isStart() or record.isFinish() or record.isSetMetadata():
            continue
        entry = record.getEntry()
        if entry not in entry_map:
            continue
        topic = entry_map[entry]
        timestamp_s = record.getTimestamp() / 1_000_000.0
 
        if topic == ENABLED_KEY:
            enabled = record.getBoolean()
            continue
 
        if not enabled:
            continue
 
        if topic == BROWNOUT_KEY:
            if record.getBoolean():
                brownout_events.append(timestamp_s)
 
        elif topic == BATTERY_VOLTAGE_KEY:
            voltage = record.getDouble()
            if voltage < BROWNOUT_VOLTAGE_THRESHOLD:
                battery_voltage_dips.append((timestamp_s, voltage))
 
        elif topic in topic_to_indexer:
            name = topic_to_indexer[topic]
            value = record.getDouble()
            indexer_series[name].append((timestamp_s, value))
 
        elif topic in topic_to_flywheel:
            name = topic_to_flywheel[topic]
            value = record.getDouble()
            flywheel_series[name].append((timestamp_s, value))
 
    # Check 1: Brownout
    if brownout_events:
        report.add(CheckResult(
            name="Brownout",
            passed=False,
            message=f"Brownout occurred {len(brownout_events)} time(s) during enabled play "
                    f"(first at t={brownout_events[0]:.2f}s)",
        ))
    else:
        report.add(CheckResult(
            name="Brownout",
            passed=True,
            message="No brownouts detected during enabled play",
        ))
 
    # Check 2: Battery voltage dip
    if battery_voltage_dips:
        min_voltage = min(v for _, v in battery_voltage_dips)
        min_time = next(t for t, v in battery_voltage_dips if v == min_voltage)
        report.add(CheckResult(
            name="Battery Voltage",
            passed=False,
            message=f"Battery voltage dipped below {BROWNOUT_VOLTAGE_THRESHOLD}V during enabled play "
                    f"(min {min_voltage:.2f}V at t={min_time:.2f}s)",
        ))
    else:
        report.add(CheckResult(
            name="Battery Voltage",
            passed=True,
            message=f"Battery voltage stayed above {BROWNOUT_VOLTAGE_THRESHOLD}V during enabled play",
        ))
 
    # Check 3: Indexer flickering
    for motor_name, series in indexer_series.items():
        flicker_windows: list[float] = []
        for i in range(1, len(series)):
            t_prev, v_prev = series[i - 1]
            t_curr, v_curr = series[i]
            # Count reversals within FLICKER_WINDOW_SECONDS
            if abs(v_prev) < FLICKER_MIN_VELOCITY or abs(v_curr) < FLICKER_MIN_VELOCITY:
                continue
            if (v_prev > 0) != (v_curr > 0):
                # Direction changed — look back to count reversals in window
                window_start = t_curr - FLICKER_WINDOW_SECONDS
                reversals = sum(
                    1
                    for j in range(1, i + 1)
                    if series[j][0] >= window_start
                    and abs(series[j - 1][1]) >= FLICKER_MIN_VELOCITY
                    and abs(series[j][1]) >= FLICKER_MIN_VELOCITY
                    and (series[j - 1][1] > 0) != (series[j][1] > 0)
                )
                if reversals >= FLICKER_MIN_REVERSALS:
                    flicker_windows.append(t_curr)
 
        if flicker_windows:
            report.add(CheckResult(
                name=f"Indexer Flicker ({motor_name})",
                passed=False,
                message=f"{len(flicker_windows)} flicker window(s) detected "
                        f"(first at t={flicker_windows[0]:.2f}s)",
            ))
        else:
            report.add(CheckResult(
                name=f"Indexer Flicker ({motor_name})",
                passed=True,
                message="No flickering detected",
            ))
 
    # Check 4: Flywheel direction mismatch
    # Left and right flywheels should spin in opposite directions (mirrored mechanically)
    left_series = flywheel_series.get("Flywheel Left", [])
    right_series = flywheel_series.get("Flywheel Right", [])
 
    if not left_series or not right_series:
        report.add(CheckResult(
            name="Flywheel Direction",
            passed=False,
            message="Missing flywheel velocity data in log",
        ))
    else:
        # Build a time-aligned comparison
        right_dict = {t: v for t, v in right_series}
        right_times = sorted(right_dict.keys())
        mismatches: list[float] = []
 
        for t_left, v_left in left_series:
            if abs(v_left) < FLYWHEEL_VELOCITY_MIN:
                continue
            # Find closest right sample
            closest = min(right_times, key=lambda t: abs(t - t_left))
            if abs(closest - t_left) > 0.1:
                continue
            v_right = right_dict[closest]
            if abs(v_right) < FLYWHEEL_VELOCITY_MIN:
                continue
            # They should spin the same direction (same sign)
            if (v_left > 0) != (v_right > 0):
                mismatches.append(t_left)
 
        if mismatches:
            report.add(CheckResult(
                name="Flywheel Direction",
                passed=False,
                message=f"Flywheel direction mismatch detected {len(mismatches)} time(s) "
                        f"(first at t={mismatches[0]:.2f}s)",
            ))
        else:
            report.add(CheckResult(
                name="Flywheel Direction",
                passed=True,
                message="Flywheels spinning in correct opposing directions",
            ))
 
    return report