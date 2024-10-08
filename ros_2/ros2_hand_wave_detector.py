import cv2
import mediapipe as mp
import numpy as np
import time
import os
import math
import subprocess
import rclpy
from rclpy.node import Node

# Initialize MediaPipe Pose solution
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=False, model_complexity=1, enable_segmentation=False, min_detection_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

# Constants for hand-waving detection
TIME_LIMIT = 1  # Max time for movement in one direction
WAVE_COUNT = 2  # Number of back and forth movements to consider it a wave
BUFFER_SIZE = 40  # Buffer size to track hand movements
VISIBILITY_THRESHOLD = 0.5

# Buffer for storing x-positions and time for each hand
hand_buffers_right = []
hand_buffers_left = []

class HandWaveDetectionNode(Node):
    def __init__(self):
        super().__init__('hand_wave_detection_node')

        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera.")
            return

        self.initial_positions = {'left': None, 'right': None}
        self.movement_initialized = {'left': False, 'right': False}

        self.process_frames()

    def process_frames(self):
        while self.cap.isOpened():
            success, frame = self.cap.read()
            if not success:
                self.get_logger().error("Failed to read from camera.")
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(frame_rgb)

            if results.pose_landmarks:
                mp_draw.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
                h, w, _ = frame.shape

                # Extract wrist and shoulder landmarks
                right_wrist = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST]
                left_wrist = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST]
                right_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
                left_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
                
                right_visibility = right_wrist.visibility
                left_visibility = left_wrist.visibility
                
                right_wrist_x, right_wrist_y = int(right_wrist.x * w), int(right_wrist.y * h)
                left_wrist_x, left_wrist_y = int(left_wrist.x * w), int(left_wrist.y * h)
                right_shoulder_x, right_shoulder_y = int(right_shoulder.x * w), int(right_shoulder.y * h)
                left_shoulder_x, left_shoulder_y = int(left_shoulder.x * w), int(left_shoulder.y * h)

                shoulder_distance = abs(right_shoulder_x - left_shoulder_x)
                proximity_threshold = shoulder_distance * 0.8
                noise_threshold = shoulder_distance * 0.01
                min_distance = shoulder_distance * 0.020

                if self.is_within_proximity(right_wrist_x, right_wrist_y, right_shoulder_x, right_shoulder_y, proximity_threshold, right_visibility, "right"):
                    self.movement_initialized['right'] = True
                    hand_buffers_right.append(right_wrist_x)
                else:
                    hand_buffers_right.clear()

                if self.is_within_proximity(left_wrist_x, left_wrist_y, left_shoulder_x, left_shoulder_y, proximity_threshold, left_visibility, "left"):
                    self.movement_initialized['left'] = True
                    hand_buffers_left.append(left_wrist_x)
                else:
                    hand_buffers_left.clear()

                if len(hand_buffers_right) > BUFFER_SIZE:
                    hand_buffers_right.pop(0)
                if len(hand_buffers_left) > BUFFER_SIZE:
                    hand_buffers_left.pop(0)

                if len(hand_buffers_right) > 1:
                    if self.waving_detection(hand_buffers_right, noise_threshold, min_distance, 'right'):
                        self.get_logger().info('Right hand wave detected! Initiating next node...')
                        self.initiate_next_node()
                        break

                if len(hand_buffers_left) > 1:
                    if self.waving_detection(hand_buffers_left, noise_threshold, min_distance, 'left'):
                        self.get_logger().info('Left hand wave detected! Initiating next node...')
                        self.initiate_next_node()
                        break

            cv2.imshow('Hand Waving Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def is_within_proximity(self, wrist_x, wrist_y, shoulder_x, shoulder_y, proximity_threshold, visibility, which_hand):
        if visibility < VISIBILITY_THRESHOLD:
            return False
        distance = math.sqrt((wrist_y - shoulder_y) ** 2 + (wrist_x - shoulder_x) ** 2)
        return distance <= proximity_threshold

    def waving_detection(self, hand_buffer, noise_threshold, min_distance, which_hand):
        wave_count = 0
        last_direction = 0
        last_position_before_direction_change = hand_buffer[0]
        movement_start_time = time.time()

        for i in range(1, len(hand_buffer)):
            current_position = hand_buffer[i]
            previous_position = hand_buffer[i - 1]
            direction = np.sign(current_position - previous_position)

            if abs(current_position - previous_position) < noise_threshold:
                continue

            if direction != last_direction:
                distance = abs(current_position - last_position_before_direction_change)
                time_elapsed = time.time() - movement_start_time
                if distance >= min_distance and time_elapsed <= TIME_LIMIT:
                    wave_count += 1
                    last_direction = direction
                    last_position_before_direction_change = current_position
                    movement_start_time = time.time()

                if wave_count >= WAVE_COUNT:
                    return True
        return False

    def initiate_next_node(self):
        subprocess.run(["ros2", "run", "your_package_name", "chan_full_version.py"])

def main(args=None):
    rclpy.init(args=args)
    node = HandWaveDetectionNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
