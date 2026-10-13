"""XLeRobot mod101 arm configuration; common sensor/navigation launch defaults."""
from xlerobot_sim.bringup import generate_launch_description as common_launch


def generate_launch_description():
    return common_launch('mod101')
