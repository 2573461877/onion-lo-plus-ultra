#!/usr/bin/env python3
"""Record Linux process and system resource usage without external packages."""

import argparse
import csv
import glob
import os
import statistics
import time
from pathlib import Path


def find_pid(pattern):
    own_pid = os.getpid()
    candidates = []
    for entry in glob.glob("/proc/[0-9]*/cmdline"):
        pid = int(entry.split("/")[2])
        if pid == own_pid:
            continue
        try:
            command = Path(entry).read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if pattern in command:
            candidates.append(pid)
    return min(candidates) if candidates else None


def read_process(pid):
    stat_text = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    remaining = stat_text[stat_text.rfind(")") + 2:].split()
    ticks = int(remaining[11]) + int(remaining[12])
    status = {}
    for line in Path(f"/proc/{pid}/status").read_text(
            encoding="utf-8").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            status[key] = value.strip()

    def status_kb(key):
        value = status.get(key, "0 kB").split()[0]
        return int(value)

    return {
        "ticks": ticks,
        "rss_kb": status_kb("VmRSS"),
        "vsize_kb": status_kb("VmSize"),
        "threads": int(status.get("Threads", "0")),
    }


def read_mem_available_kb():
    for line in Path("/proc/meminfo").read_text(
            encoding="utf-8").splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1])
    return 0


def read_max_temperature_c():
    temperatures = []
    for path in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
        try:
            value = float(Path(path).read_text(encoding="utf-8").strip())
        except (OSError, TypeError, ValueError):
            continue
        temperatures.append(value / 1000.0 if value > 1000.0 else value)
    return max(temperatures) if temperatures else float("nan")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--match", default="onion_lo_plus_node")
    parser.add_argument("--output", required=True)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    args = parser.parse_args()
    if args.interval <= 0.0 or args.wait_timeout <= 0.0:
        parser.error("--interval and --wait-timeout must be positive")

    wait_deadline = time.monotonic() + args.wait_timeout
    pid = None
    while time.monotonic() < wait_deadline and pid is None:
        pid = find_pid(args.match)
        if pid is None:
            time.sleep(min(args.interval, 0.2))
    if pid is None:
        raise SystemExit(
            f"process containing {args.match!r} was not found")

    clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    start_monotonic = time.monotonic()
    previous_time = start_monotonic
    previous_ticks = None
    rows = []
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as stream:
        fieldnames = [
            "epoch_sec", "elapsed_sec", "pid", "cpu_percent",
            "rss_kb", "vsize_kb", "threads", "mem_available_kb",
            "load1", "max_temperature_c",
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        while True:
            now = time.monotonic()
            try:
                process = read_process(pid)
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                break
            if previous_ticks is None:
                cpu_percent = 0.0
            else:
                cpu_percent = (
                    (process["ticks"] - previous_ticks) / clock_ticks
                    / max(now - previous_time, 1e-9) * 100.0
                )
            row = {
                "epoch_sec": time.time(),
                "elapsed_sec": now - start_monotonic,
                "pid": pid,
                "cpu_percent": cpu_percent,
                "rss_kb": process["rss_kb"],
                "vsize_kb": process["vsize_kb"],
                "threads": process["threads"],
                "mem_available_kb": read_mem_available_kb(),
                "load1": os.getloadavg()[0],
                "max_temperature_c": read_max_temperature_c(),
            }
            writer.writerow(row)
            stream.flush()
            rows.append(row)
            previous_ticks = process["ticks"]
            previous_time = now
            time.sleep(args.interval)

    measured = rows[1:] if len(rows) > 1 else rows
    cpu_values = [row["cpu_percent"] for row in measured]
    rss_values = [row["rss_kb"] for row in measured]
    temperature_values = [
        row["max_temperature_c"] for row in measured
        if row["max_temperature_c"] == row["max_temperature_c"]
    ]
    print(
        "samples={} pid={} cpu_mean_percent={:.2f} "
        "cpu_peak_percent={:.2f} rss_peak_kb={} "
        "temperature_peak_c={}".format(
            len(rows),
            pid,
            statistics.fmean(cpu_values) if cpu_values else 0.0,
            max(cpu_values) if cpu_values else 0.0,
            max(rss_values) if rss_values else 0,
            "{:.2f}".format(max(temperature_values))
            if temperature_values else "unavailable",
        )
    )


if __name__ == "__main__":
    main()
