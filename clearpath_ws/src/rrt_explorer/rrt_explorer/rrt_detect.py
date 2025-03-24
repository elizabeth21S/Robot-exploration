#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import PoseStamped

from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import math

# ---------------------------------------
# 1. Inicializar el detector HOG de OpenCV
# ---------------------------------------
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

def non_max_suppression_fast(boxes, overlap_thresh=0.5):
    if len(boxes) == 0:
        return []

    if boxes.dtype.kind == "i":
        boxes = boxes.astype("float")

    pick = []

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    idxs = np.argsort(y2)

    while len(idxs) > 0:
        last = len(idxs) - 1
        i = idxs[last]
        pick.append(i)

        xx1 = np.maximum(x1[i], x1[idxs[:last]])
        yy1 = np.maximum(y1[i], y1[idxs[:last]])
        xx2 = np.minimum(x2[i], x2[idxs[:last]])
        yy2 = np.minimum(y2[i], y2[idxs[:last]])

        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)

        overlap = (w * h) / areas[idxs[:last]]

        idxs = np.delete(
            idxs,
            np.concatenate(([last], np.where(overlap > overlap_thresh)[0]))
        )

    return boxes[pick].astype("int")


class CameraLidarPersonDetectorNode(Node):
    def __init__(self):
        super().__init__('camera_lidar_person_detector_node')

        self.image_topic = '/a200_0000/sensors/camera_0/color/image'
        self.depth_topic = '/a200_0000/sensors/camera_0/depth/image'
        self.camera_info_topic = '/a200_0000/sensors/camera_0/color/camera_info'
        self.lidar_topic = '/a200_0000/sensors/lidar2d_0/scan'

        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, 10)
        self.create_subscription(LaserScan, self.lidar_topic, self.lidar_callback, 10)

        self.bridge = CvBridge()
        self.last_depth_image = None
        self.camera_info = None

        self.person_goal_pub = self.create_publisher(PoseStamped, '/a200_0000/person_goal', 10)
        self.side_offset = 0.5

        self.fig, self.ax = plt.subplots()
        self.cluster_plot_initialized = False

        self.get_logger().info("Camera and Lidar Person Detector Node iniciado.")

    def info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        self.last_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def lidar_callback(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        points = np.array([
            [r * math.cos(a), r * math.sin(a)]
            for r, a in zip(msg.ranges, angles)
            if 0.2 < r < 5.0
        ])

        if len(points) < 5:
            return

        clustering = DBSCAN(eps=0.2, min_samples=4).fit(points)
        labels = clustering.labels_

        self.ax.clear()
        unique_labels = set(labels)
        for label in unique_labels:
            cluster_points = points[labels == label]
            color = 'red' if label != -1 else 'gray'
            self.ax.scatter(cluster_points[:, 0], cluster_points[:, 1], c=color, s=25, label=f'Cluster {label}')

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('LIDAR Clusters DBSCAN')
        self.ax.legend()
        self.ax.grid(True)
        plt.pause(0.01)

    def image_callback(self, msg):
        if self.camera_info is None or self.last_depth_image is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image_flipped = cv2.flip(cv_image, -1)

        boxes, weights = hog.detectMultiScale(
            cv_image_flipped,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05
        )

        filtered_boxes = []
        MIN_WEIGHT = 0.5
        for i, (x, y, w, h) in enumerate(boxes):
            if weights[i] < MIN_WEIGHT:
                continue
            aspect_ratio = float(h) / float(w)
            if aspect_ratio < 1.2 or aspect_ratio > 4.0:
                continue
            filtered_boxes.append([x, y, x + w, y + h])

        filtered_boxes = np.array(filtered_boxes)
        final_boxes = non_max_suppression_fast(filtered_boxes, 0.4)

        for (x1, y1, x2, y2) in final_boxes:
            w = x2 - x1
            h = y2 - y1
            cx = x1 + w // 2
            cy = y1 + h // 2

            cv2.rectangle(cv_image_flipped, (x1, y1), (x2, y2), (0, 255, 0), 2)

            depth_value = self.last_depth_image[cy, cx]
            if np.isnan(depth_value) or depth_value <= 0:
                continue

            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx_intr = self.camera_info.k[2]
            cy_intr = self.camera_info.k[5]

            X_cam = (cx - cx_intr) * depth_value / fx
            Y_cam = (cy - cy_intr) * depth_value / fy

            side = self.side_offset if Y_cam >= 0 else -self.side_offset

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'a200_0000/base_link'
            goal_pose.header.stamp = msg.header.stamp
            goal_pose.pose.position.x = X_cam
            goal_pose.pose.position.y = Y_cam + side
            goal_pose.pose.orientation.w = 1.0

            self.person_goal_pub.publish(goal_pose)
            self.get_logger().info(
                f"Persona detectada y publicando objetivo en X={X_cam:.2f}, Y={Y_cam + side:.2f}"
            )

        cv2.imshow("Camera View Flipped (HOG detections)", cv_image_flipped)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarPersonDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

