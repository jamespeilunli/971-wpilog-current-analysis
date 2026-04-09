# WPILOG Current Analysis

Summarizes subsystem current draw from an AdvantageKit/WPILOG file and splits the results into:

- `enabled`
- `auto`
- `teleop`

It writes a CSV and, unless disabled, a very plain HTML report with PNG charts.
The HTML report includes:

- Phase toggle: `enabled`, `auto`, `teleop`
- Metric toggle: `Supply Current`, `Stator Current`
- Smaller graphs sized for a normal browser window
- Top 5 direct-mechanism current time-series plots
- Full CSV data rendered into readable tables

## Usage

Default run:

```bash
uv run wpilog-current-analysis /path/to/log.wpilog
```

CSV only:

```bash
uv run wpilog-current-analysis /path/to/log.wpilog --no-report
```

Choose the number of current buckets:

```bash
uv run wpilog-current-analysis /path/to/log.wpilog --quantiles 8
```

Choose output paths:

```bash
uv run wpilog-current-analysis /path/to/log.wpilog -o /path/to/output.csv --report-dir /path/to/report_dir
```

## Outputs

- Default CSV: `generated/<log_name>/<log_name>_current_summary.csv`
- Default report directory: `generated/<log_name>/report`
- Report entrypoint: `<report_dir>/index.html`

## Important Columns

- `phase`: `enabled`, `auto`, or `teleop`
- `phase_seconds`: Time spent in that phase with valid accumulation
- `average`: Time-weighted average current in that phase
- `top_bucket_avg`: Average current in the highest configured bucket
- `bucket_*_avg`: One column per configured bucket
- `member_topics_expected`: How many motor topics the subsystem expects from the robot code mapping
- `member_topics_present`: How many of those topics were actually found in the log
