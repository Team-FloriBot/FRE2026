import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import numpy as np
from enum import Enum
from dataclasses import dataclass
import re

@dataclass
class PerceptionData: # Platzhalter, Variablen durch später sinnvoll genutzte ersetzen
    # --- Lidar Daten ---
    left_dist: float = 0.0
    right_dist: float = 0.0
    center_error: float = 0.0
    row_detected: bool = False
    row_end_detected: bool = False
    
    # --- IMU & Odometrie ---
    current_yaw: float = 0.0      # Aktueller Winkel in Radiant
    dist_moved: float = 0.0       # Seit dem letzten Reset gefahrene Strecke
    
    # --- State-spezifisch ---
    is_row_edge_detected: bool = False # Für COUNTING_ROWS (Seitliche Erkennung)


@dataclass
class ControlCommand:
    linear: float
    angular: float


# Diese Klasse übersetzt Rohdaten in das PerceptionData Objekt.
class Perception:
    def __init__(self, x_min, x_max, y_min, y_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

    def process(self, cloud_msg, imu_msg, odom_msg, current_state) -> PerceptionData:
        data = PerceptionData()
        # To-Do: Fusioniere IMU (Orientierung) und Odometrie (Distanz) in data
        
        # Abhängig vom State werden unterschiedliche Merkmale extrahiert:
        if current_state == State.DRIVE_IN_ROW:
            # To-Do: Berechne center_error und row_end_detected (Lidar vorne/seitlich)
            pass
        elif current_state == State.EXIT_ROW:
            # To-Do: Prüfe, ob Distanz zu den Reihen hinter uns groß genug ist
            pass
        elif current_state == State.TURN:
            # To-Do: Überwache nur den relativen Winkel (IMU)
            pass
        elif current_state == State.COUNTING_ROWS:
            # To-Do: Erkenne "Eingänge" der Reihen (Lidar seitlich)
            pass
        elif current_state == State.ENTER_ROW:
            # To-Do: Erkenne, ob der Roboter wieder "gerade" in der Reihe steht
            pass
            
        return data
    

class State(Enum):
    DRIVE_IN_ROW = 1
    EXIT_ROW = 2
    TURN = 3
    COUNTING_ROWS = 4
    ENTER_ROW = 5

# extrahiert das zu fahrende Pattern aus dem string und speichert es als Liste
class Pattern:
    def __init__(self, pattern_str):
        self.steps = self.parse(pattern_str)
        self.index = 0

    def parse(self, pattern_str):
        tokens = pattern_str.split("-")
        result = []
        for t in tokens:
            match = re.match(r"(\d+)([LR])", t.strip())
            if match:
                count = int(match.group(1))
                direction = match.group(2)
                result.append((count, direction))
        return result

    def current(self):
        if self.index < len(self.steps):
            return self.steps[self.index]
        return None

    def next(self):
        self.index += 1

class StateMachine:
    def __init__(self, pattern):
        self.state = State.DRIVE_IN_ROW
        self.pattern = pattern
        self.target_yaw = 0.0
        self.rows_passed = 0

    def update(self, perception: PerceptionData):
        # Hier jeweils Bedingungen festlegen, wann States gewechselt werden, hier nur Vorschläge wie das aussehen könnte
        if self.state == State.DRIVE_IN_ROW:
            # To-Do: Wechsel zu EXIT_ROW, wenn perception.row_end_detected True
            pass

        elif self.state == State.EXIT_ROW:
            # To-Do: Fahre X Meter aus der Reihe (Odometrie), dann Wechsel zu TURN
            pass

        elif self.state == State.TURN:
            # To-Do: Wenn Winkel (perception.current_yaw) Ziel erreicht -> COUNTING_ROWS
            # Hinweis: Target_yaw muss beim Eintritt in TURN einmalig gesetzt werden
            pass

        elif self.state == State.COUNTING_ROWS:
            # To-Do: Zähle Reihen. Wenn rows_passed == pattern.count -> ENTER_ROW
            pass

        elif self.state == State.ENTER_ROW:
            # To-Do: Wenn Ausrichtung stimmt und Roboter tief genug in Reihe -> DRIVE_IN_ROW
            # Hier Pattern-Index erhöhen!
            pass

# Berechnet die tatsächlichen cmd_vel Befehle.
class Controller:
    def compute(self, state, perception, direction): # jede folgende Funktion gibt übereinen Regler einen cmd_vel Befehl zurück
        if state == State.DRIVE_IN_ROW:
            return self.drive_in_row(perception)
        
        elif state == State.EXIT_ROW:
            return self.exit_row(perception)
            
        elif state == State.TURN:
            return self.turn(perception)
            
        elif state == State.COUNTING_ROWS:
            return self.counting_rows(perception)
            
        elif state == State.ENTER_ROW:
            return self.enter_row(perception)

    def drive_in_row(self, perception):
        # To-Do: Implementiere Spurhalte-Logik
        pass

    def exit_row(self, perception):
        # To-Do: Fahre langsam geradeaus aus der Reihe
        pass

    def turn(self, perception):
        # To-Do: Drehe auf der Stelle (P-Regler für den Winkel)
        pass

    def counting_rows(self, perception):
        # To-Do: Fahre parallel zum Vorgewende (Headland)
        pass

    def enter_row(self, perception):
        # To-Do: Kurvenfahrt in die Zielreihe
        pass

# ROS Node
class FieldRobotNavigator(Node):
    def __init__(self):
        super().__init__("maize_navigator")

        # -------------------------
        # Parameter deklarieren, hier anpassen je nach obiger Implementierung, welche Parameter brauchen wir?
        # -------------------------
        self.declare_parameter("pattern", "1L-1R-2L-3R")

        self.declare_parameter("perception.x_min", 0.0)
        self.declare_parameter("perception.x_max", 2.0)
        self.declare_parameter("perception.y_min", 0.1)
        self.declare_parameter("perception.y_max", 1.0)

        self.declare_parameter("controller.kp", 2.0)
        self.declare_parameter("controller.kd", 0.5)
        self.declare_parameter("controller.ki", 0.0)

        self.declare_parameter("topics.pointcloud", "/point_cloud")
        self.declare_parameter("topics.cmd_vel", "/cmd_vel")

        # -------------------------
        # Parameter holen
        # -------------------------
        pattern_str = self.get_parameter("pattern").get_parameter_value().string_value

        x_min = self.get_parameter("perception.x_min").value
        x_max = self.get_parameter("perception.x_max").value
        y_min = self.get_parameter("perception.y_min").value
        y_max = self.get_parameter("perception.y_max").value

        kp = self.get_parameter("controller.kp").value
        kd = self.get_parameter("controller.kd").value
        ki = self.get_parameter("controller.ki").value

        pointcloud_topic = self.get_parameter("topics.pointcloud").value
        cmd_vel_topic = self.get_parameter("topics.cmd_vel").value

        # -------------------------
        # Module initialisieren
        # -------------------------
        self.perception = Perception(x_min, x_max, y_min, y_max)

        self.pattern = Pattern(pattern_str)
        self.state_machine = StateMachine(self.pattern)

        self.controller = Controller()
        self.controller.kp = kp
        self.controller.kd = kd
        self.controller.ki = ki

        self.latest_cloud = None

        # -------------------------
        # ROS Schnittstellen
        # -------------------------
        self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.cloud_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.timer = self.create_timer(0.1, self.loop)

        # -------------------------
        # Parameter-Update Callback (optional, aber stark!)
        # -------------------------
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("FieldRobotNavigator gestartet")

    # -------------------------
    # Dynamische Parameteränderung
    # -------------------------
    def parameter_callback(self, params):
        for param in params:
            if param.name == "controller.kp":
                self.controller.kp = param.value
            elif param.name == "controller.kd":
                self.controller.kd = param.value
            elif param.name == "controller.ki":
                self.controller.ki = param.value

        return rclpy.parameter.SetParametersResult(successful=True)

    # -------------------------
    def cloud_callback(self, msg):
        self.latest_cloud = msg

    # -------------------------
    def loop(self):
        if self.latest_cloud is None:
            return

        perception = self.perception.process(self.latest_cloud)
        state = self.state_machine.update(perception)

        direction = self.state_machine.current_direction
        cmd = self.controller.compute(state, perception, direction)

        twist = Twist()
        twist.linear.x = cmd.linear
        twist.angular.z = cmd.angular

        self.cmd_pub.publish(twist)

        self.get_logger().debug(f"State: {state}, cmd: {cmd}")


# main
def main(args=None):
    rclpy.init(args=args)
    node = FieldRobotNavigator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

