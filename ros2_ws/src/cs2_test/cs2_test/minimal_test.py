"""
Takeoff, fly a square, land.
Safe for real Crazyflie with Crazyswarm2 + mocap.
"""

from crazyflie_py import Crazyswarm
import numpy as np


TAKEOFF_HEIGHT = 1.0
TAKEOFF_DURATION = 2.5

SIDE_LENGTH = 0.5      # meters
LEG_DURATION = 3.0     # seconds per side

HOVER_AFTER_TAKEOFF = 1.0
HOVER_BEFORE_LAND = 1.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    # --- Takeoff ---
    cf.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_AFTER_TAKEOFF)

    # Reference position (world frame)
    start_pos = np.array(cf.initialPosition) + np.array([0.0, 0.0, TAKEOFF_HEIGHT])

    # Define square corners relative to start
    square_offsets = [
        np.array([0.0, 0.0, 0.0]),
        np.array([SIDE_LENGTH, 0.0, 0.0]),
        np.array([SIDE_LENGTH, SIDE_LENGTH, 0.0]),
        np.array([0.0, SIDE_LENGTH, 0.0]),
        np.array([0.0, 0.0, 0.0]),  # return to start
    ]

    # --- Fly square ---
    for offset in square_offsets:
        target_pos = start_pos + offset
        cf.goTo(target_pos, yaw=0.0, duration=LEG_DURATION)
        timeHelper.sleep(LEG_DURATION + 0.5)

    # --- Land ---
    timeHelper.sleep(HOVER_BEFORE_LAND)
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3.0)