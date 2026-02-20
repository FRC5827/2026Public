"""
FRC Driver Station Events Parser

Parses .dsevents files and reports robot events, loop overruns with per-subsystem
timing breakdowns, and errors/warnings.

Usage:
    python tools/parse_dsevents.py <path_to_dsevents_file>
    python tools/parse_dsevents.py   # auto-detects .dsevents in logs/latest/
    python tools/parse_dsevents.py --verbose  # print full event timeline too
"""

import re
import sys
import os
import argparse
from collections import defaultdict

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LATEST_DIR = os.path.join(REPO_ROOT, "logs", "latest")


def find_dsevents_file():
    """Find a .dsevents file to parse. Prefers logs/latest/, then parent dir."""
    for search_dir in [LATEST_DIR, REPO_ROOT]:
        if not os.path.isdir(search_dir):
            continue
        for f in sorted(os.listdir(search_dir)):
            if f.endswith(".dsevents"):
                return os.path.join(search_dir, f)
    return None


def extract_events(data):
    """Extract (time_str, message) pairs from the .dsevents binary format.

    The file contains binary record headers with embedded text blobs.
    Each text blob may contain one or more structured log entries of the form:
        <TagVersion>1 <time> MM.SSS <message> text here
    We extract all ASCII runs and parse the structured tags."""
    text_runs = re.findall(rb"[ -~]{10,}", data)
    combined = b" ".join(text_runs).decode("ascii", errors="replace")
    return re.findall(
        r"<time>\s+([\d.]+)\s+<message>\s+(.+?)(?=<TagVersion>|$)", combined
    )


def parse_secs(s):
    """Extract the first float seconds value from a string like '0.173368s'."""
    m = re.search(r"([\d.]+)s", s)
    return float(m.group(1)) if m else None


def analyze(events):
    """Break events down into startup, overruns, subsystem timings, and errors."""
    startup = []
    overrun_cycles = []  # list of {t, total_ms, subsystems: {name: ms}}
    errors = []  # (t, message) for non-overrun errors/warnings

    # Accumulate Tracer epoch lines between robotPeriodic() markers
    pending_epochs = {}  # subsystem_name -> ms

    STARTUP_PATTERNS = [
        "Robot program starting",
        "Robot program startup complete",
        "[AdvantageKit]",
        "NT: Listening",
        "photonvision",
        "AdvantageScope",
        "Elastic",
        "FMS Connected",
        "FRC Driver Station",
    ]

    # Tracer epoch pattern: any message containing "SomeName(): 0.123456s"
    # These appear both as plain lines and with a "Warning at ...Tracer..." prefix.
    # WPILib prints one epoch line per subsystem, ending with the robotPeriodic() total.
    # Epochs arrive BEFORE their corresponding robotPeriodic line in the event stream,
    # so we accumulate them and save when we see robotPeriodic.
    EPOCH_RE = re.compile(r"([\w.]+\(\))\s*:\s*([\d.]+)s")

    for t_str, msg in events:
        t = float(t_str)
        msg = msg.strip()

        # --- Startup events ---
        if any(p in msg for p in STARTUP_PATTERNS):
            startup.append((t, msg))
            continue

        # --- Tracer epoch lines ---
        epoch_m = EPOCH_RE.search(msg)
        if epoch_m:
            name = epoch_m.group(1)
            ms = float(epoch_m.group(2)) * 1000
            if "robotPeriodic" in name:
                # This line carries the total for the current cycle.
                # Save epochs accumulated since the last robotPeriodic as this cycle.
                overrun_cycles.append(
                    {"t": t, "total_ms": ms, "subsystems": dict(pending_epochs)}
                )
                pending_epochs = {}
            else:
                pending_epochs[name] = ms
            continue

        # --- WPILib loop overrun notice / CommandScheduler overrun (noise) ---
        if (
            "printLoopOverrunMessage" in msg
            or "Loop time of" in msg
            or "CommandScheduler loop overrun" in msg
        ):
            continue

        # --- Errors / warnings ---
        if (
            "error" in msg.lower()
            or "Error" in msg
            or "Warning" in msg
            or "exception" in msg.lower()
        ) and "Tracer" not in msg and "printLoop" not in msg:
            errors.append((t, msg))

    return startup, overrun_cycles, errors


def subsystem_stats(overrun_cycles):
    """Compute per-subsystem max and over-20ms count across all overrun cycles."""
    stats = defaultdict(lambda: {"max_ms": 0.0, "over20": 0, "over5": 0})
    for cycle in overrun_cycles:
        for name, ms in cycle["subsystems"].items():
            stats[name]["max_ms"] = max(stats[name]["max_ms"], ms)
            if ms > 20:
                stats[name]["over20"] += 1
            if ms > 5:
                stats[name]["over5"] += 1
    return dict(stats)


def print_report(path, events, verbose=False):
    startup, overrun_cycles, errors = analyze(events)

    print(f"File: {os.path.basename(path)}")
    print(f"Total events: {len(events)}")

    # --- Startup ---
    print()
    print("=== STARTUP SEQUENCE ===")
    seen_startup = set()
    for t, msg in startup:
        key = msg[:60]
        if key not in seen_startup:
            seen_startup.add(key)
            print(f"  t={t:7.1f}s  {msg[:120]}")

    # --- Loop overruns ---
    print()
    print("=== LOOP OVERRUN SUMMARY ===")
    if not overrun_cycles:
        print("  No loop overrun breakdowns found.")
    else:
        print(f"  {len(overrun_cycles)} cycles with Tracer breakdown reported")
        print()
        print("  Worst robotPeriodic() cycles:")
        worst = sorted(overrun_cycles, key=lambda c: -(c["total_ms"] or 0))[:10]
        for c in worst:
            parts = ", ".join(
                f"{n}: {ms:.0f}ms"
                for n, ms in sorted(c["subsystems"].items(), key=lambda x: -x[1])
                if ms > 2
            )
            print(f"    t={c['t']:7.1f}s  total={c['total_ms']:.0f}ms  [{parts}]")

        print()
        print("  Per-subsystem worst times (across all overrun cycles):")
        stats = subsystem_stats(overrun_cycles)
        for name, s in sorted(stats.items(), key=lambda x: -x[1]["max_ms"]):
            bar = "#" * min(40, int(s["max_ms"] / 10))
            print(
                f"    {name:<35} max={s['max_ms']:6.0f}ms  "
                f">20ms={s['over20']:3d}  >5ms={s['over5']:3d}  {bar}"
            )

    # --- Errors ---
    print()
    print("=== ERRORS / WARNINGS ===")
    if not errors:
        print("  None.")
    else:
        # Deduplicate: strip trailing single-char binary garbage before keying
        buckets = defaultdict(list)
        for t, msg in errors:
            # Normalize key: take up to 80 chars, strip trailing whitespace/junk
            key = re.sub(r"\s{2,}\S$", "", msg[:80]).strip()
            buckets[key].append(t)
        for key, times in sorted(buckets.items(), key=lambda x: x[1][0]):
            count = len(times)
            first_t = times[0]
            suffix = f"  (x{count}, first t={first_t:.1f}s)" if count > 1 else f"  (t={first_t:.1f}s)"
            print(f"  {key[:120]}{suffix}")

    # --- Verbose: full event timeline ---
    if verbose:
        print()
        print("=== FULL EVENT LOG ===")
        for t, msg in events:
            print(f"  t={float(t):7.1f}s  {msg[:140]}")


def main():
    parser = argparse.ArgumentParser(
        description="Parse FRC Driver Station .dsevents files."
    )
    parser.add_argument(
        "file", nargs="?", help=".dsevents file to parse (default: auto-detect)"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Print full event timeline"
    )
    args = parser.parse_args()

    path = args.file
    if not path:
        path = find_dsevents_file()
        if not path:
            print("Error: No .dsevents file found. Specify a path or run pull_logs.py first.")
            sys.exit(1)
        print(f"Auto-detected: {path}")

    if not os.path.exists(path):
        print(f"Error: File not found: {path}")
        sys.exit(1)

    with open(path, "rb") as f:
        data = f.read()

    events = extract_events(data)
    if not events:
        print("No events found in file.")
        sys.exit(0)

    print_report(path, events, verbose=args.verbose)


if __name__ == "__main__":
    main()
