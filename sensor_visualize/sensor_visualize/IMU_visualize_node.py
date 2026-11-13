# --------------------------------------------------------------------------------
# IMU 3D-Visualisierung (ROS2 + Pygame)
# 
# Diese Anwendung abonniert IMU-Daten (Yaw, Pitch, Roll) über ROS2
# und visualisiert die Ausrichtung eines Körpers in 3D mit Pygame.
# Die IMU-Daten werden als String über den Topic "imu_topic" gesendet.
# Die Visualisierung nutzt eine einfache Drahtgitter-Box mit Perspektivprojektion.
# --------------------------------------------------------------------------------

import sys
import math
import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pygame.locals import *
from operator import itemgetter
import time

class Point3D:
    def __init__(self, x=0, y=0, z=0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def rotateX(self, angle):
        # Rotation um X-Achse (Pitch)
        rad = angle
        cosa = math.cos(rad)
        sina = math.sin(rad)
        y = self.y * cosa - self.z * sina
        z = self.y * sina + self.z * cosa
        return Point3D(self.x, y, z)

    def rotateY(self, angle):
        # Rotation um Y-Achse (Yaw)
        rad = angle
        cosa = math.cos(rad)
        sina = math.sin(rad)
        z = self.z * cosa - self.x * sina
        x = self.z * sina + self.x * cosa
        return Point3D(x, self.y, z)

    def rotateZ(self, angle):
        # Rotation um Z-Achse (Roll)
        rad = angle
        cosa = math.cos(rad)
        sina = math.sin(rad)
        x = self.x * cosa - self.y * sina
        y = self.x * sina + self.y * cosa
        return Point3D(x, y, self.z)

    def project(self, win_width, win_height, fov, viewer_distance):
        # Perspektivische Projektion von 3D auf 2D
        factor = fov / (viewer_distance + self.z)
        x = self.x * factor + win_width / 2
        y = -self.y * factor + win_height / 2
        return Point3D(x, y, self.z)

class Simulation(Node):
    def __init__(self, win_width=1080, win_height=640):
        super().__init__('imu_visualization_node')

        # ROS2 Subscriber
        self.subscription = self.create_subscription(
            String,
            'imu/ypr',
            self.listener_callback,
            10)

        pygame.init()
        self.screen = pygame.display.set_mode((win_width, win_height))
        pygame.display.set_caption("Visualization of IMU Orientation")
        self.clock = pygame.time.Clock()

        # Eckpunkte des Körpers (Box)
        self.vertices = [
            Point3D(-1, -0.2, 1),
            Point3D(1, -0.2, 1),
            Point3D(1, -0.2, -1),
            Point3D(-1, -0.2, -1),
            Point3D(-1, 0.2, 1),
            Point3D(1, 0.2, 1),
            Point3D(1, 0.2, -1),
            Point3D(-1, 0.2, -1)
        ]

        # Flächenindizes und Farben
        self.faces = [(0,1,2,3), (1,5,6,2), (5,4,7,6), (4,0,3,7), (0,4,5,1), (3,2,6,7)]
        self.colors = [(0,0,255), (0,255,0), (0,0,255), (0,255,0), (255,255,0), (255,255,0)]

        self.imu_data = [0.0, 0.0, 0.0]  # [yaw, pitch, roll]

    def listener_callback(self, msg):
        try:
            # Daten aus dem ROS2-Topic in float konvertieren
            self.imu_data = [float(val) for val in msg.data.split()]
        except ValueError as e:
            self.get_logger().error(f"Fehler beim Verarbeiten der IMU-Daten: {e}")

    def run(self):
        nowP = time.time()
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            self.screen.fill((0, 32, 0))
            t = []

            angleX = math.radians(self.imu_data[1])  # pitch
            angleY = math.radians(self.imu_data[0])  # yaw
            angleZ = math.radians(self.imu_data[2])  # roll

            for v in self.vertices:
                # Rotationen anwenden (Reihenfolge ist wichtig)
                r = v.rotateX(-angleX)
                r1 = r.rotateY(-angleY)  # ggf. invertieren je nach Koordinatensystem
                #r1 = r.rotateY(angleY)
                r2 = r1.rotateZ(-angleZ)

                # Projektion auf 2D
                p = r2.project(self.screen.get_width(), self.screen.get_height(), 256, 4)
                t.append(p)

            # Z-Werte pro Fläche berechnen für Tiefensortierung (Painter's Algorithmus)
            avg_z = []
            for i, f in enumerate(self.faces):
                z = sum(t[j].z for j in f) / 4
                avg_z.append([i, z])

            # Flächen zeichnen, sortiert nach Entfernung
            for tmp in sorted(avg_z, key=itemgetter(1), reverse=True):
                face_index = tmp[0]
                f = self.faces[face_index]
                pointlist = [(t[f[i]].x, t[f[i]].y) for i in range(4)]
                pygame.draw.polygon(self.screen, self.colors[face_index], pointlist)

            pygame.display.flip()

            # Debug-Ausgabe alle 0.5 Sekunden
            elapsed = time.time() - nowP
            if elapsed > 0.5:
                print("Yaw = ", self.imu_data[0], " Pitch = ", self.imu_data[1], " Roll = ", self.imu_data[2])
                nowP = time.time()

def main(args=None):
    rclpy.init(args=args)
    simulation = Simulation()

    try:
        simulation.run()
    except KeyboardInterrupt:
        pass
    finally:
        simulation.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == "__main__":
    main()
