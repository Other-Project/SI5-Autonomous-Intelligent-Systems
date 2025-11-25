import math
import numpy as np

# Excitation: closer → faster
def tf_linear_excitation(distance_m: float, MAX_WHEEL_SPEED: float, k: float) -> float:
    """
    Excitatory transfer function: closer object -> higher speed
    distance_m: distance observed at one angle (meters)
    MAX_WHEEL_SPEED: maximum wheel speed (m/s)
    k : decreasing coefficient
        
    Returns wheel speed in m/s.
    """
    if distance_m <= 0.0 or distance_m == float('inf'):
        return 0.0

    distance_mm = distance_m * 1000.0
    
    # linear excitation: speed decreases linearly with distance
    v = MAX_WHEEL_SPEED * np.exp(-k * distance_mm)
    v = max(0.0, min(v, MAX_WHEEL_SPEED))

    return v

# Inhibition: closer → slower
def tf_linear_inhibition(distance_m: float, MAX_WHEEL_SPEED: float, k: float) -> float:
    """
    Inhibitory transfer function: closer object -> lower speed
    distance_m: distance observed at one angle (meters)
    MAX_WHEEL_SPEED: maximum wheel speed (m/s)
        
    Returns wheel speed in m/s.
    """
    if distance_m <= 0.0 or distance_m == float('inf'):
        return MAX_WHEEL_SPEED  

    # convert distance to mm
    distance_mm = distance_m * 1000.0
    
    # linear inhibition: speed increases linearly with distance
    v = MAX_WHEEL_SPEED * (1 - np.exp(-k * distance_mm))

    v = max(0.0, min(v, MAX_WHEEL_SPEED))
    return v    

