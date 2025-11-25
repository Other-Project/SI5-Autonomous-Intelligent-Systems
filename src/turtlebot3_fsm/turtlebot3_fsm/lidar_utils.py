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

def get_index_for_angle(scan: LaserScan, angle_deg: float) -> int:
    """
    Returns the index in scan.ranges corresponding to the given angle in degrees.
    Handles wrap-around (e.g. -10° == 350° if scan covers 0→360°).
    """
    if scan is None or not scan.ranges:
        return 0

    angle_rad = math.radians(angle_deg)

    # Wrap angle into [angle_min, angle_max)
    two_pi = 2 * math.pi
    angle_span = scan.angle_max - scan.angle_min
    if angle_span > 2 * math.pi - 0.1:  # full 360° scan
        while angle_rad < scan.angle_min:
            angle_rad += two_pi
        while angle_rad >= scan.angle_max:
            angle_rad -= two_pi

    idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
    idx = max(0, min(len(scan.ranges) - 1, idx))  # clamp

    return idx

def get_avg_range_at_angle(scan: LaserScan, angle_deg: float, window_deg: float = 2.0):
    """
    Compute the average valid range in ±window_deg around the given angle (in degrees).
    Returns:
        (avg_range, (idx_min, idx_max), window_ranges)
    where:
        - avg_range: average valid distance
        - (idx_min, idx_max): index window bounds (wrapped if needed)
        - window_ranges: list of (index, range_value) pairs for that angular window
    """
    if scan is None or not scan.ranges:
        return float('inf'), (0, 0), []

    n = len(scan.ranges)

    # Base indices for target and window
    idx_center = get_index_for_angle(scan, angle_deg)
    idx_p = get_index_for_angle(scan, angle_deg + window_deg)
    idx_n = get_index_for_angle(scan, angle_deg - window_deg)

    # Build index window (handles wrap-around)
    if idx_n <= idx_p:
        indices = list(range(idx_n, idx_p + 1))
    else:
        # Window crosses 0° boundary
        indices = list(range(idx_n, n)) + list(range(0, idx_p + 1))

    # Extract valid ranges
    window_ranges = []
    valid_values = []
    for i in indices:
        r = scan.ranges[i]
        if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
            window_ranges.append((i, r))
            valid_values.append(r)
        else:
            window_ranges.append((i, float('inf')))

    avg_range = sum(valid_values) / len(valid_values) if valid_values else float('inf')

    return avg_range, window_ranges

def get_min_range_at_angle(scan: LaserScan, angle_deg: float, window_deg: float = 2.0):
    """
    Compute the minimum valid range in ±window_deg around the given angle (in degrees).
    Returns:
        (min_range, (idx_min, idx_max), window_ranges)
    where:
        - min_range: smallest valid distance
        - (idx_min, idx_max): index window bounds (wrapped if needed)
        - window_ranges: list of (index, range_value) pairs for that angular window
    """
    if scan is None or not scan.ranges:
        return float('inf'), (0, 0), []

    n = len(scan.ranges)

    # Base indices for target and window
    idx_center = get_index_for_angle(scan, angle_deg)
    idx_p = get_index_for_angle(scan, angle_deg + window_deg)
    idx_n = get_index_for_angle(scan, angle_deg - window_deg)

    # Build index window (handles wrap-around)
    if idx_n <= idx_p:
        indices = list(range(idx_n, idx_p + 1))
    else:
        # Window crosses 0° boundary
        indices = list(range(idx_n, n)) + list(range(0, idx_p + 1))

    # Extract valid ranges
    window_ranges = []
    valid_values = []
    for i in indices:
        r = scan.ranges[i]
        if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
            window_ranges.append((i, r))
            valid_values.append(r)
        else:
            window_ranges.append((i, float('inf')))

    min_range = min(valid_values) if valid_values else float('inf')

    return min_range, window_ranges
    

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
    for (i, r) in index_ranges:
        ranges[i] = r

    debug_scan.ranges = ranges

    publisher.publish(debug_scan)

def get_angle_of_clear_area(scan: LaserScan, dist_min: float, window_deg: float = 5.0) -> float | None:
    """
    Returns the relative angle (in radians) of the first window ±window_deg
    where all ranges are greater than dist_min. Returns None if no such window exists.
    """
    if scan is None or not scan.ranges:
        return None

    n = len(scan.ranges)

    for i in range(n):
        angle_deg = math.degrees(scan.angle_min + i * scan.angle_increment)

        _, window_ranges = get_min_range_at_angle(scan, angle_deg, window_deg)
        
        if all(r > dist_min for _, r in window_ranges):
            # Convert index en angle relatif
            return scan.angle_min + i * scan.angle_increment

    return None

def get_angle_of_clear_area_world(scan: LaserScan, current_yaw: float, min_dist: float, window_deg: float = 5.0) -> float:
    """
    Returns the best direction (absolute angle in world frame) combining current yaw and
    the clearest relative angle from the scan.
    """
    rel_angle = get_angle_of_clear_area(scan, min_dist, window_deg)
    if rel_angle is None:
        return current_yaw  # stay on current heading if no data

    return normalize_angle(current_yaw + rel_angle)


def normalize_angle(angle: float) -> float:
    """
    Normalize an angle to [-pi, pi].
    """
    return math.atan2(math.sin(angle), math.cos(angle))
    
