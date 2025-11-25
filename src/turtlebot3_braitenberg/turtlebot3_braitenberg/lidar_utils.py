import math
from sensor_msgs.msg import LaserScan

def get_max_range(scan: LaserScan) -> float:
    """
    Return the maximum valid range (distance) detected by the LiDAR.
    Ignores NaN, inf, and values outside [range_min, range_max].
    """
    valid_ranges = [r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
    if not valid_ranges:
        return 0.0
    return max(valid_ranges)


def get_min_range(scan: LaserScan) -> float:
    """
    Return the minimum valid range (distance) detected by the LiDAR.
    Ignores NaN, inf, and values outside [range_min, range_max].
    """
    valid_ranges = [r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
    if not valid_ranges:
        return 0.0
    return min(valid_ranges)


def get_avg_range_at_angle(scan: LaserScan, angle_deg: float, window_deg: float = 2.0) -> float:
    """
    Compute average range (distance) and return indices corresponding to angle_deg +/- window_deg.
    """
    angle_rad = math.radians(angle_deg)
    half_window = math.radians(window_deg)

    idx_min = max(0, int(round((angle_rad - half_window - scan.angle_min) / scan.angle_increment)))
    idx_max = min(len(scan.ranges) - 1, int(round((angle_rad + half_window - scan.angle_min) / scan.angle_increment)))

    values = [r for r in scan.ranges[idx_min:idx_max+1] if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
    avg_range = float('inf') if not values else sum(values) / len(values)

    return avg_range, (idx_min, idx_max)

def get_min_range_at_angle(scan: LaserScan, angle_deg: float, window_deg: float = 2.0) -> float:
    """
    Compute min range (distance) and return indices corresponding to angle_deg +/- window_deg.
    """
    angle_rad = math.radians(angle_deg)
    half_window = math.radians(window_deg)

    idx_min = max(0, int(round((angle_rad - half_window - scan.angle_min) / scan.angle_increment)))
    idx_max = min(len(scan.ranges) - 1, int(round((angle_rad + half_window - scan.angle_min) / scan.angle_increment)))

    values = [r for r in scan.ranges[idx_min:idx_max+1] if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
    min_range = float('inf') if not values else min(values)

    return min_range, (idx_min, idx_max)

def publish_debug_scan(original_scan: LaserScan, index_ranges, publisher):
    """
    Publish a LaserScan with only the ranges in indices.
    Other values are set to +inf.
    """
    debug_scan = LaserScan()
    debug_scan.header = original_scan.header
    debug_scan.angle_min = original_scan.angle_min
    debug_scan.angle_max = original_scan.angle_max
    debug_scan.angle_increment = original_scan.angle_increment
    debug_scan.time_increment = original_scan.time_increment
    debug_scan.scan_time = original_scan.scan_time
    debug_scan.range_min = original_scan.range_min
    debug_scan.range_max = original_scan.range_max

    # Crée une copie "vide" de ranges
    ranges = [float('inf')] * len(original_scan.ranges)

    # Remplit uniquement les zones d’intérêt
    for (start, end) in index_ranges:
        # Clamp pour éviter un dépassement d’indice
        start = max(0, start)
        end = min(len(original_scan.ranges) - 1, end)
        for i in range(start, end + 1):
            ranges[i] = original_scan.ranges[i]

    debug_scan.ranges = ranges

    publisher.publish(debug_scan)

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    return angle
    
def get_angle_of_clear_area(scan: LaserScan, window_deg: float = 5.0):
    """
    Returns (relative_angle, avg_range) corresponding to the index whose average range readings 
    within ±window_deg are maximal.
    """
    if scan is None:
        return (None, 0.0)

    ranges = list(scan.ranges)
    n = len(ranges)
    if n == 0:
        return (None, 0.0)

    angle_inc = scan.angle_increment
    if abs(angle_inc) < 1e-9:
        return (0.0, ranges[0] if ranges else 0.0)

    half_window = int(round(math.radians(window_deg) / abs(angle_inc)))
    if half_window < 0:
        half_window = 0

    vals = [0.0] * n
    counts = [0] * n
    for i, r in enumerate(ranges):
        if r is None or math.isinf(r) or math.isnan(r) or r <= 0.0:
            vals[i] = 0.0
            counts[i] = 0
        else:
            vals[i] = float(r)
            counts[i] = 1

    prefix_val = [0.0] * (n + 1)
    prefix_count = [0] * (n + 1)
    for i in range(n):
        prefix_val[i+1] = prefix_val[i] + vals[i]
        prefix_count[i+1] = prefix_count[i] + counts[i]

    best_avg = -1.0
    best_idx = 0
    for i in range(n):
        start = max(0, i - half_window)
        end = min(n - 1, i + half_window)
        sum_win = prefix_val[end+1] - prefix_val[start]
        cnt_win = prefix_count[end+1] - prefix_count[start]
        if cnt_win > 0:
            avg = sum_win / cnt_win
        else:
            avg = 0.0

        if avg > best_avg:
            best_avg = avg
            best_idx = i

    best_angle = scan.angle_min + best_idx * angle_inc
    return (best_angle, best_avg)

def get_angle_of_clear_area_world(scan: LaserScan, current_yaw, window_deg=5.0):
    rel_angle, _ = get_angle_of_clear_area(scan, window_deg)
    if rel_angle is None:
        return 0.0
    return normalize_angle(current_yaw + rel_angle)    

    
    
