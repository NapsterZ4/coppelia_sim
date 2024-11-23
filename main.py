import robotica
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class FuzzyRules:
    """Fuzzy rules for the robot navigation"""

    VERY_CLOSE = 0.8
    CLOSE = 0.5
    SAFE = 0.3

    NORMAL_SPEED = 1.0
    TURN_SPEED = 0.6
    SLOW_SPEED = 0.4


class SensorReader:
    """Class to interpret sensor data"""

    @staticmethod
    def get_danger_zones(readings: List[float]) -> Tuple[float, float, float]:
        """
        Get the danger level in three zones: left, center and right
        Returns: (left_danger, center_danger, right_danger)
        """
        left_danger = min(readings[2:4])
        center_danger = min(readings[4:6])
        right_danger = min(readings[6:8])

        return left_danger, center_danger, right_danger


class FuzzyController:
    """Fuzzy controller for the robot navigation"""

    def __init__(self):
        self.rules = FuzzyRules()
        self.sensor_reader = SensorReader()

    def evaluate_situation(self, left: float, center: float,
                           right: float) -> str:
        if center < self.rules.VERY_CLOSE:
            return "emergency_turn"
        elif left < self.rules.CLOSE:
            return "turn_right"
        elif right < self.rules.CLOSE:
            return "turn_left"
        else:
            return "advance"

    def get_wheel_speeds(self, action: str) -> Tuple[float, float]:
        speeds = {
            "advance": (self.rules.NORMAL_SPEED, self.rules.NORMAL_SPEED),
            "turn_left": (self.rules.SLOW_SPEED, self.rules.TURN_SPEED),
            "turn_right": (self.rules.TURN_SPEED, self.rules.SLOW_SPEED),
            "emergency_turn": (-self.rules.SLOW_SPEED, self.rules.TURN_SPEED)
        }
        return speeds[action]

    def compute_movement(self, readings: List[float]) -> Tuple[float, float]:
        left, center, right = self.sensor_reader.get_danger_zones(readings)
        action = self.evaluate_situation(left, center, right)
        return self.get_wheel_speeds(action)


def run_robot():
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    controller = FuzzyController()

    try:
        coppelia.start_simulation()
        while coppelia.is_running():
            readings = robot.get_sonar()
            left_speed, right_speed = controller.compute_movement(readings)
            robot.set_speed(left_speed, right_speed)
    finally:
        coppelia.stop_simulation()


if __name__ == '__main__':
    run_robot()
