"""
FRC Driver Station Log (.dslog) Parser
Parses binary .dslog v4 files and reports overrun analysis, CPU/CAN stats, and trip time distribution.

Usage:
    python parse_dslog.py <path_to_dslog_file>
    python parse_dslog.py                         # uses default path if no argument given
"""

import struct
import sys
import os

def parse_dslog(path):
    if not os.path.exists(path):
        print(f'Error: File not found: {path}')
        return

    with open(path, 'rb') as f:
        data = f.read()

    print(f'File: {os.path.basename(path)}')
    print(f'File size: {len(data)} bytes')
    version = struct.unpack('>i', data[:4])[0]
    print(f'Version: {version}')

    if version != 4:
        print(f'Warning: Expected version 4, got {version}. Results may be incorrect.')

    record_size = 10
    num_records = (len(data) - 4) // record_size
    print(f'Number of records: {num_records}')
    print(f'Duration: ~{num_records * 0.02:.1f}s ({num_records * 0.02 / 60:.1f} min)')

    overrun_count = 0
    max_trip = 0
    trip_times = []
    high_cpu_count = 0
    high_can_count = 0
    cpu_vals = []
    can_vals = []

    for i in range(num_records):
        offset = 4 + i * record_size
        record = data[offset:offset + record_size]
        if len(record) < record_size:
            break

        # dslog v4 record format (10 bytes):
        # Byte 0: Trip time (value / 2.0 = ms)
        # Byte 1: Packet loss (0-100)
        # Bytes 2-3: Voltage (big-endian uint16 / 256.0)
        # Byte 4: roboRIO CPU %
        # Byte 5: Status byte (brownout, watchdog, DS teleop/auto/disabled, estop)
        # Byte 6: CAN utilization (value * 100.0 / 255.0 = %)
        # Byte 7: Wifi signal dB
        # Bytes 8-9: Bandwidth (big-endian uint16)
        trip_time_ms = record[0] / 2.0
        cpu = record[4]
        can_util = record[6] * 100.0 / 255.0

        trip_times.append(trip_time_ms)
        cpu_vals.append(cpu)
        can_vals.append(can_util)
        if trip_time_ms > max_trip:
            max_trip = trip_time_ms
        if trip_time_ms > 20:
            overrun_count += 1
        if cpu > 80:
            high_cpu_count += 1
        if can_util > 70:
            high_can_count += 1

    if not trip_times:
        print('No records found.')
        return

    pct_overrun = overrun_count * 100.0 / num_records
    pct_cpu = high_cpu_count * 100.0 / num_records
    pct_can = high_can_count * 100.0 / num_records

    print(f'\n=== OVERRUN ANALYSIS ===')
    print(f'Total records: {num_records}')
    print(f'Overruns (>20ms trip): {overrun_count} ({pct_overrun:.1f}%)')
    print(f'Max trip time: {max_trip:.1f}ms')
    print(f'High CPU (>80%): {high_cpu_count} ({pct_cpu:.1f}%)')
    print(f'High CAN (>70%): {high_can_count} ({pct_can:.1f}%)')

    brackets = [0, 5, 10, 15, 20, 25, 30, 40, 50, 100, 200]
    print(f'\nTrip time distribution:')
    for j in range(len(brackets) - 1):
        count = sum(1 for t in trip_times if brackets[j] <= t < brackets[j + 1])
        pct = count * 100.0 / len(trip_times)
        bar = '#' * int(pct)
        print(f'  {brackets[j]:3d}-{brackets[j+1]:3d}ms: {count:6d} ({pct:5.1f}%) {bar}')
    count_big = sum(1 for t in trip_times if t >= 200)
    print(f'  200+ms:  {count_big:6d} ({count_big * 100.0 / len(trip_times):.1f}%)')

    print(f'\nWorst 20 trip times:')
    indexed = [(t, i) for i, t in enumerate(trip_times)]
    indexed.sort(reverse=True)
    for t, idx in indexed[:20]:
        offset2 = 4 + idx * record_size
        rec = data[offset2:offset2 + record_size]
        cpu2 = rec[4]
        can2 = rec[6] * 100.0 / 255.0
        volt2 = struct.unpack('>H', rec[2:4])[0] / 256.0
        status2 = rec[5]
        time_sec = idx * 0.02
        print(f'  t={time_sec:7.1f}s  trip={t:6.1f}ms  CPU={cpu2:3d}%  CAN={can2:4.1f}%  V={volt2:5.2f}  status=0x{status2:02x}')

    print(f'\nConsecutive overrun streaks (>20ms):')
    streak = 0
    max_streak = 0
    streak_start = 0
    streaks = []
    for i, t in enumerate(trip_times):
        if t > 20:
            if streak == 0:
                streak_start = i
            streak += 1
        else:
            if streak > 2:
                streaks.append((streak, streak_start, i - 1))
            if streak > max_streak:
                max_streak = streak
            streak = 0
    if streak > 2:
        streaks.append((streak, streak_start, len(trip_times) - 1))
    streaks.sort(reverse=True)
    for s, start, end in streaks[:10]:
        print(f'  {s} consecutive overruns at t={start * 0.02:.1f}s - {end * 0.02:.1f}s')
    print(f'  Max consecutive: {max_streak}')

    avg_trip = sum(trip_times) / len(trip_times)
    sorted_trips = sorted(trip_times)
    print(f'\nAverage trip time: {avg_trip:.2f}ms')
    print(f'Median trip time: {sorted_trips[len(trip_times) // 2]:.1f}ms')
    print(f'95th percentile: {sorted_trips[int(len(trip_times) * 0.95)]:.1f}ms')
    print(f'99th percentile: {sorted_trips[int(len(trip_times) * 0.99)]:.1f}ms')

    avg_cpu = sum(cpu_vals) / len(cpu_vals)
    avg_can = sum(can_vals) / len(can_vals)
    print(f'\nAverage CPU: {avg_cpu:.1f}%')
    print(f'Max CPU: {max(cpu_vals)}%')
    print(f'Average CAN: {avg_can:.1f}%')
    print(f'Max CAN: {max(can_vals):.1f}%')


if __name__ == '__main__':
    if len(sys.argv) > 1:
        dslog_path = sys.argv[1]
    else:
        # Default: find first .dslog in parent directory
        search_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        dslog_files = [f for f in os.listdir(search_dir) if f.endswith('.dslog')]
        if dslog_files:
            dslog_path = os.path.join(search_dir, dslog_files[0])
            print(f'Auto-detected: {dslog_path}')
        else:
            print('Usage: python parse_dslog.py <path_to_dslog_file>')
            sys.exit(1)

    parse_dslog(dslog_path)
