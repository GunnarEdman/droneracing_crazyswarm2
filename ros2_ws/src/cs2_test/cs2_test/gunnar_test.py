"""Fly around in a square at a set altitude and with set dimensions"""

from crazyflie_py import Crazyswarm
import math

TAKEOFF_DURATION = 2.5
Z_HOVER = 1.0
TBW_DURATION = 5.0  # Time Between Waypoints

square_points = [
    [1.0, 0.0, Z_HOVER],
    [1.0, 1.0, Z_HOVER],
    [0.0, 1.0, Z_HOVER],
    [0.0, 0.0, Z_HOVER]
]
# waypoints = [
#     # x, y, z, yaw
#     [1.0, 1.0, Z_HOVER, math.pi/4],
#     [-1.0, 1.0, Z_HOVER, math.pi],
#     [-1.0, -1.0, Z_HOVER,-math.pi/2],
#     [1.0, -1.0, Z_HOVER, 0.0],
#     [0.0, 0.0, Z_HOVER, 0.0]
# ]


def main():
    # Init swarm
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    # =============== Flight maneuvers =================
    # Takeoff
    cf.takeoff(targetHeight=Z_HOVER, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)
    # cf.goTo([0.0, 0.0, Z_HOVER], math.pi/4, 1.0)
    # timeHelper.sleep(TAKEOFF_DURATION)

    # Square pattern
    for pos in square_points:
        # cf.goTo(goal, yaw, duration, relative=False)
        print(f"Going to {pos}")
        cf.goTo(pos, 0.0, TBW_DURATION)
        timeHelper.sleep(TBW_DURATION + 0.5) # Wait for the move to finish

    # Landing
    cf.land(targetHeight=0.04, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == '__main__':
    main()
