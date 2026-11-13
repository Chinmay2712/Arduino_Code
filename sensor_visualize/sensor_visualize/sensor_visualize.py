import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import pygame
import sys
import math
import numpy as np
from pygame.locals import *

# 3D point class
class Point3D:
    def __init__(self, x=0, y=0, z=0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def transform(self, matrix):
        v = np.array([self.x, self.y, self.z])
        x, y, z = matrix @ v
        return Point3D(x, y, z)

    def project(self, win_width, win_height, fov, viewer_distance):
        factor = fov / (viewer_distance + self.z)
        x = self.x * factor + win_width / 2
        y = -self.y * factor + win_height / 2
        return Point3D(x, y, self.z)


class QuaternionVisualizer(Node):
    def __init__(self, win_width=1080, win_height=640):
        super().__init__('quaternion_visualizer')

        # Subscribe to IMU quaternion
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.listener_callback,
            10)

        pygame.init()
        self.screen = pygame.display.set_mode((win_width, win_height))
        pygame.display.set_caption("Quaternion-based IMU Visualization")

        self.q = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z

        # Cube vertices
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

        self.faces = [(0,1,2,3),(1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7)]
        self.colors = [(0,0,255),(0,255,0),(0,0,255),(0,255,0),(255,255,0),(255,255,0)]

    def listener_callback(self, msg):
        self.q = [
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ]

    def quaternion_to_matrix(self, q):
        w, x, y, z = q
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),       2*(x*z + y*w)],
            [2*(x*y + z*w),         1 - 2*(x*x + z*z),   2*(y*z - x*w)],
            [2*(x*z - y*w),         2*(y*z + x*w),       1 - 2*(x*x + y*y)]
        ])

    def run(self):
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            self.screen.fill((0, 32, 0))

            # Convert quaternion to rotation matrix
            R = self.quaternion_to_matrix(self.q)

            # Transform vertices
            t = []
            for v in self.vertices:
                rotated = v.transform(R)
                projected = rotated.project(
                    self.screen.get_width(),
                    self.screen.get_height(),
                    256,
                    4
                )
                t.append(projected)

            # Painter’s algorithm
            avg_z = []
            for i, f in enumerate(self.faces):
                z = (t[f[0]].z + t[f[1]].z + t[f[2]].z + t[f[3]].z) / 4
                avg_z.append([i, z])

            for face_index, _ in sorted(avg_z, key=lambda x: x[1], reverse=True):
                f = self.faces[face_index]
                pointlist = [(t[f[0]].x, t[f[0]].y),
                             (t[f[1]].x, t[f[1]].y),
                             (t[f[2]].x, t[f[2]].y),
                             (t[f[3]].x, t[f[3]].y)]
                pygame.draw.polygon(self.screen, self.colors[face_index], pointlist)

            pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    snode = QuaternionVisualizer()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()
        
if __name__ == "__main__":
    main()
    
