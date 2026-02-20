"""
FRC Log Puller - Downloads AdvantageKit logs from the roboRIO USB stick.

Downloads .wpilog files from the roboRIO, skips already-downloaded files,
and manages USB space with a smart retention algorithm:
  - Always keeps the most recent N logs on the USB (default 10)
  - Deletes oldest already-downloaded logs when free space drops below threshold
  - Never deletes a log that hasn't been successfully downloaded first
  - Automatically matches downloaded .wpilog files with Driver Station .dslog files

Usage:
    python tools/pull_logs.py                # download new logs, clean up if needed
    python tools/pull_logs.py --list         # list logs on the roboRIO USB
    python tools/pull_logs.py --all          # force re-download all logs
    python tools/pull_logs.py --no-cleanup   # download without cleaning USB
    python tools/pull_logs.py --keep 20      # keep last 20 logs on USB (default 10)
    python tools/pull_logs.py --min-free 1024 # minimum free MB on USB (default 512)
    python tools/pull_logs.py --no-match     # skip DS log matching

Requires: SSH access to roboRIO (built into Windows 10+)
"""

import subprocess
import sys
import os
import argparse
import re
import shutil
from datetime import datetime, timedelta, timezone

# ---- Configuration ----
TEAM_NUMBER = 9317
ROBORIO_USER = "lvuser"
ROBORIO_HOSTS = [
    f"10.{TEAM_NUMBER // 100}.{TEAM_NUMBER % 100}.2",
    f"roborio-{TEAM_NUMBER}-frc.local",
    "172.22.11.2",  # USB direct connection
]
REMOTE_LOG_DIR = "/U/logs"
LOCAL_LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
LATEST_DIR = os.path.join(LOCAL_LOG_DIR, "latest")

# Known DS log locations (searched in order)
DS_LOG_DIRS = [
    os.path.join(os.environ.get("PUBLIC", r"C:\Users\Public"),
                 "Documents", "FRC", "Log Files", "DSLogs"),
    os.path.join(os.environ.get("USERPROFILE", ""),
                 "Documents", "FRC", "Log Files", "DSLogs"),
]

SSH_OPTS = [
    "-o", "StrictHostKeyChecking=no",
    "-o", "UserKnownHostsFile=/dev/null",
    "-o", "LogLevel=ERROR",
    "-o", "ConnectTimeout=5",
    "-o", "BatchMode=yes",
]

DEFAULT_KEEP_RECENT = 10      # always keep this many recent logs on USB
DEFAULT_MIN_FREE_MB = 1024    # start cleaning when free space drops below this
MATCH_TOLERANCE_SECS = 120    # max seconds apart to consider a wpilog/dslog match


def ssh_cmd(host, command, timeout=10):
    """Run a command on the roboRIO via SSH. Returns (success, stdout)."""
    full_cmd = ["ssh"] + SSH_OPTS + [f"{ROBORIO_USER}@{host}", command]
    try:
        result = subprocess.run(
            full_cmd, capture_output=True, text=True, timeout=timeout
        )
        return result.returncode == 0, result.stdout.strip()
    except subprocess.TimeoutExpired:
        return False, ""
    except FileNotFoundError:
        print("Error: ssh not found. Ensure OpenSSH is installed (Windows 10+).")
        sys.exit(1)


def scp_download(host, remote_path, local_path, timeout=120):
    """Download a file from the roboRIO via SCP. Returns success."""
    full_cmd = ["scp"] + SSH_OPTS + [f"{ROBORIO_USER}@{host}:{remote_path}", local_path]
    try:
        result = subprocess.run(full_cmd, capture_output=True, text=True, timeout=timeout)
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print(f"  Timeout downloading {os.path.basename(remote_path)}")
        return False


def find_roborio():
    """Try each known roboRIO address and return the first that responds."""
    print("Searching for roboRIO...")
    for host in ROBORIO_HOSTS:
        ok, _ = ssh_cmd(host, "echo ok", timeout=5)
        if ok:
            print(f"  Connected to {host}")
            return host
    return None


def list_remote_logs(host):
    """List .wpilog files on the USB with sizes. Returns list of (filename, size_bytes)."""
    ok, output = ssh_cmd(host, f"ls -l {REMOTE_LOG_DIR}/*.wpilog 2>/dev/null", timeout=10)
    if not ok or not output:
        return []

    logs = []
    for line in output.strip().split("\n"):
        parts = line.split()
        if len(parts) >= 9 and parts[-1].endswith(".wpilog"):
            size = int(parts[4])
            filename = os.path.basename(parts[-1])
            logs.append((filename, size))

    logs.sort(key=lambda x: x[0])
    return logs


def get_usb_free_space(host):
    """Get free space on the USB stick in MB."""
    ok, output = ssh_cmd(host, f"df -m {REMOTE_LOG_DIR} | tail -1", timeout=10)
    if not ok or not output:
        return None
    parts = output.split()
    if len(parts) >= 4:
        try:
            return int(parts[3])
        except ValueError:
            pass
    return None


def format_size(size_bytes):
    """Format bytes as human-readable string."""
    if size_bytes >= 1024 * 1024 * 1024:
        return f"{size_bytes / (1024**3):.1f} GB"
    elif size_bytes >= 1024 * 1024:
        return f"{size_bytes / (1024**2):.1f} MB"
    elif size_bytes >= 1024:
        return f"{size_bytes / 1024:.1f} KB"
    return f"{size_bytes} B"


# ---- Timestamp parsing ----

def parse_wpilog_timestamp(filename):
    """Parse UTC datetime from a wpilog filename like 'akit_26-02-06_04-46-53.wpilog'.
    Returns a timezone-aware UTC datetime, or None if the filename doesn't match."""
    m = re.match(r"akit_(\d{2})-(\d{2})-(\d{2})_(\d{2})-(\d{2})-(\d{2})\.wpilog$", filename)
    if not m:
        return None
    yy, mo, dd, hh, mi, ss = (int(x) for x in m.groups())
    try:
        return datetime(2000 + yy, mo, dd, hh, mi, ss, tzinfo=timezone.utc)
    except ValueError:
        return None


def parse_dslog_timestamp(filename):
    """Parse local datetime from a dslog filename like '2026_02_05 20_46_37 Thu.dslog'.
    Returns a naive local datetime, or None if the filename doesn't match."""
    m = re.match(r"(\d{4})_(\d{2})_(\d{2}) (\d{2})_(\d{2})_(\d{2}) \w+\.dslog$", filename)
    if not m:
        return None
    yr, mo, dd, hh, mi, ss = (int(x) for x in m.groups())
    try:
        return datetime(yr, mo, dd, hh, mi, ss)
    except ValueError:
        return None


def find_ds_log_dir():
    """Find the DS log directory on this machine. Returns path or None."""
    for d in DS_LOG_DIRS:
        if os.path.isdir(d):
            return d
    return None


def get_dslog_files(ds_dir):
    """Return list of (filename, full_path) for .dslog files in the given directory."""
    try:
        return [
            (f, os.path.join(ds_dir, f))
            for f in os.listdir(ds_dir)
            if f.endswith(".dslog")
        ]
    except OSError:
        return []


def match_logs(wpilog_filenames):
    """Match wpilog files to their corresponding dslog files by timestamp.
    Returns a dict of {wpilog_filename: dslog_full_path} for matched pairs.
    Silently returns empty dict if DS logs aren't available."""
    ds_dir = find_ds_log_dir()
    if not ds_dir:
        return {}

    dslog_files = get_dslog_files(ds_dir)
    if not dslog_files:
        return {}

    # Parse all dslog timestamps (local time -> UTC for comparison)
    local_tz_offset = datetime.now(timezone.utc).astimezone().utcoffset()
    dslog_parsed = []
    for dslog_name, dslog_path in dslog_files:
        ts = parse_dslog_timestamp(dslog_name)
        if ts:
            # Convert local time to UTC by subtracting the local offset
            ts_utc = ts - local_tz_offset
            dslog_parsed.append((ts_utc, dslog_name, dslog_path))

    if not dslog_parsed:
        return {}

    # Match each wpilog to closest dslog within tolerance
    matches = {}
    for wpilog_name in wpilog_filenames:
        wpi_ts = parse_wpilog_timestamp(wpilog_name)
        if not wpi_ts:
            continue

        # wpi_ts is already UTC-aware; dslog timestamps are now naive UTC
        wpi_ts_naive = wpi_ts.replace(tzinfo=None)
        best_delta = None
        best_dslog = None
        for ds_ts_utc, ds_name, ds_path in dslog_parsed:
            delta = abs((wpi_ts_naive - ds_ts_utc).total_seconds())
            if delta <= MATCH_TOLERANCE_SECS and (best_delta is None or delta < best_delta):
                best_delta = delta
                best_dslog = (ds_name, ds_path, delta)

        if best_dslog:
            matches[wpilog_name] = best_dslog  # (dslog_name, dslog_path, delta_secs)

    return matches


def print_matches(wpilog_filenames):
    """Print a table matching wpilog files to dslog files. Gracefully skips if unavailable."""
    matches = match_logs(wpilog_filenames)
    if not matches and not find_ds_log_dir():
        print("\n  (DS log directory not found on this machine - skipping match)")
        return

    if not matches:
        print("\n  (No matching DS logs found)")
        return

    print(f"\n{'  wpilog':<45} {'dslog':<45} {'offset':>6}")
    print("  " + "-" * 96)
    for wpilog_name in wpilog_filenames:
        if wpilog_name in matches:
            ds_name, ds_path, delta = matches[wpilog_name]
            print(f"  {wpilog_name:<43} {ds_name:<45} {delta:>4.0f}s")
        else:
            wpi_ts = parse_wpilog_timestamp(wpilog_name)
            if wpi_ts:
                print(f"  {wpilog_name:<43} {'(no match)':<45}")


# ---- Main commands ----

def do_list(host):
    """List logs on the roboRIO USB."""
    logs = list_remote_logs(host)
    free_mb = get_usb_free_space(host)

    if not logs:
        print("No .wpilog files found on USB.")
        return

    print(f"\nLogs on roboRIO USB ({REMOTE_LOG_DIR}):")
    print(f"{'Filename':<50} {'Size':>10}")
    print("-" * 62)
    total_size = 0
    for filename, size in logs:
        print(f"  {filename:<48} {format_size(size):>10}")
        total_size += size

    print("-" * 62)
    print(f"  {'Total:':<48} {format_size(total_size):>10}")
    if free_mb is not None:
        print(f"  {'Free space:':<48} {free_mb:>7} MB")
    print(f"  {len(logs)} log file(s)")

    # Show DS log matches
    print_matches([f for f, _ in logs])


def do_pull(host, force_all=False, cleanup=True, keep_recent=DEFAULT_KEEP_RECENT,
            min_free_mb=DEFAULT_MIN_FREE_MB, show_matches=True):
    """Download new logs and optionally clean up USB."""
    os.makedirs(LOCAL_LOG_DIR, exist_ok=True)

    remote_logs = list_remote_logs(host)
    if not remote_logs:
        print("No .wpilog files found on USB.")
        return

    # Determine which logs need downloading
    local_files = set(os.listdir(LOCAL_LOG_DIR)) if os.path.exists(LOCAL_LOG_DIR) else set()

    if force_all:
        to_download = remote_logs
    else:
        to_download = [(f, s) for f, s in remote_logs if f not in local_files]

    # Download new logs
    downloaded = []
    if to_download:
        print(f"\nDownloading {len(to_download)} new log(s) to {LOCAL_LOG_DIR}/")
        for filename, size in to_download:
            remote_path = f"{REMOTE_LOG_DIR}/{filename}"
            local_path = os.path.join(LOCAL_LOG_DIR, filename)
            print(f"  {filename} ({format_size(size)})...", end=" ", flush=True)
            if scp_download(host, remote_path, local_path):
                local_size = os.path.getsize(local_path)
                if local_size > 0:
                    print(f"OK ({format_size(local_size)})")
                    downloaded.append(filename)
                else:
                    print("FAILED (empty file)")
                    os.remove(local_path)
            else:
                print("FAILED")
        print(f"  Downloaded {len(downloaded)}/{len(to_download)} file(s)")
    else:
        print("\nAll logs already downloaded.")

    # Show DS log matches for downloaded files (or all remote if nothing new)
    if show_matches:
        match_files = downloaded if downloaded else [f for f, _ in remote_logs]
        print_matches(match_files)

    # Refresh local file list after downloads
    local_files = set(os.listdir(LOCAL_LOG_DIR)) if os.path.exists(LOCAL_LOG_DIR) else set()

    # Update logs/latest/ with the newest log set if anything was downloaded
    update_latest_dir(downloaded)

    # Cleanup phase
    if not cleanup:
        print("\nSkipping USB cleanup (--no-cleanup).")
        return

    free_mb = get_usb_free_space(host)
    if free_mb is None:
        print("\nCould not determine USB free space, skipping cleanup.")
        return

    print(f"\nUSB free space: {free_mb} MB (threshold: {min_free_mb} MB)")

    if free_mb >= min_free_mb:
        print("Sufficient space, no cleanup needed.")
        return

    # Build list of candidates for deletion:
    # - Must have been downloaded locally (verified by presence in local_files)
    # - Must NOT be in the most recent `keep_recent` logs
    # - Sorted oldest first (by filename which contains timestamp)
    remote_logs_sorted = sorted(remote_logs, key=lambda x: x[0])  # oldest first
    protected = set(f for f, _ in remote_logs_sorted[-keep_recent:])  # most recent N

    candidates = [
        (f, s) for f, s in remote_logs_sorted
        if f in local_files and f not in protected
    ]

    if not candidates:
        print(f"No logs eligible for deletion (keeping {keep_recent} most recent, "
              f"only deleting already-downloaded logs).")
        return

    print(f"Cleaning up USB (keeping {keep_recent} most recent)...")
    freed = 0
    deleted_count = 0
    for filename, size in candidates:
        if free_mb + freed / (1024 * 1024) >= min_free_mb:
            break
        remote_path = f"{REMOTE_LOG_DIR}/{filename}"
        ok, _ = ssh_cmd(host, f"rm {remote_path}", timeout=10)
        if ok:
            print(f"  Deleted {filename} ({format_size(size)})")
            freed += size
            deleted_count += 1
        else:
            print(f"  Failed to delete {filename}")

    freed_mb = freed / (1024 * 1024)
    print(f"  Freed {freed_mb:.0f} MB ({deleted_count} file(s) deleted)")
    print(f"  Estimated free space: {free_mb + freed_mb:.0f} MB")


def update_latest_dir(downloaded_filenames):
    """Copy the most recent log set into logs/latest/ when new logs are downloaded.

    Copies the newest .wpilog plus its matched .dslog and .dsevents (if found)
    into LATEST_DIR, first clearing any existing files there.
    Only runs when at least one new wpilog was downloaded this session.
    """
    if not downloaded_filenames:
        return

    # Find the most recent wpilog across all local files (filename sorts chronologically)
    local_wpilogs = sorted(
        f for f in os.listdir(LOCAL_LOG_DIR) if f.endswith(".wpilog")
    )
    if not local_wpilogs:
        return
    latest_wpilog = local_wpilogs[-1]

    # Match to a DS log by timestamp
    matches = match_logs([latest_wpilog])

    # Clear and recreate the latest directory
    if os.path.exists(LATEST_DIR):
        shutil.rmtree(LATEST_DIR)
    os.makedirs(LATEST_DIR)

    # Copy the wpilog
    shutil.copy2(
        os.path.join(LOCAL_LOG_DIR, latest_wpilog),
        os.path.join(LATEST_DIR, latest_wpilog),
    )
    print(f"\nUpdated logs/latest/ -> {latest_wpilog}")

    if latest_wpilog in matches:
        dslog_name, dslog_path, delta = matches[latest_wpilog]

        # Copy .dslog
        shutil.copy2(dslog_path, os.path.join(LATEST_DIR, dslog_name))
        print(f"  + {dslog_name}  (timestamp offset {delta:.0f}s)")

        # Copy .dsevents with the same base name if it exists
        dsevents_name = os.path.splitext(dslog_name)[0] + ".dsevents"
        dsevents_path = os.path.join(os.path.dirname(dslog_path), dsevents_name)
        if os.path.exists(dsevents_path):
            shutil.copy2(dsevents_path, os.path.join(LATEST_DIR, dsevents_name))
            print(f"  + {dsevents_name}")
    else:
        print("  (no matching DS log found — only wpilog copied)")


def main():
    parser = argparse.ArgumentParser(
        description="Pull AdvantageKit logs from the roboRIO USB stick."
    )
    parser.add_argument("--list", action="store_true", help="List logs on the roboRIO without downloading")
    parser.add_argument("--all", action="store_true", help="Re-download all logs (not just new ones)")
    parser.add_argument("--no-cleanup", action="store_true", help="Skip USB cleanup after downloading")
    parser.add_argument("--no-match", action="store_true", help="Skip DS log matching")
    parser.add_argument("--keep", type=int, default=DEFAULT_KEEP_RECENT,
                        help=f"Number of recent logs to always keep on USB (default {DEFAULT_KEEP_RECENT})")
    parser.add_argument("--min-free", type=int, default=DEFAULT_MIN_FREE_MB,
                        help=f"Minimum free space in MB before cleanup (default {DEFAULT_MIN_FREE_MB})")
    parser.add_argument("--host", type=str, default=None,
                        help="Override roboRIO hostname/IP")
    args = parser.parse_args()

    host = args.host if args.host else find_roborio()
    if not host:
        print("Error: Could not connect to roboRIO. Is the robot on and connected?")
        print(f"  Tried: {', '.join(ROBORIO_HOSTS)}")
        sys.exit(1)

    if args.list:
        do_list(host)
    else:
        do_pull(
            host,
            force_all=args.all,
            cleanup=not args.no_cleanup,
            keep_recent=args.keep,
            min_free_mb=args.min_free,
            show_matches=not args.no_match,
        )


if __name__ == "__main__":
    main()
