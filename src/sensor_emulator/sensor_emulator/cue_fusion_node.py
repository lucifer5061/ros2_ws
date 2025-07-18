#!/usr/bin/env python3

import json
import csv
import os
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

class CueFusionNode(Node):
    def __init__(self):
        super().__init__('cue_fusion_node')

        # Subscriptions
        self.vis_sub = self.create_subscription(
            String, '/mock/visual_cue', self.vis_cb, 10)
        self.rf_sub  = self.create_subscription(
            String, '/mock/rf_cue',     self.rf_cb,  10)

        # Publisher
        self.fuse_pub = self.create_publisher(
            String, '/synthetic_target_cue', 10)

        # Buffers for incoming cues
        # Each entry will be a dict containing the original fields + 'recv_ts'
        self.vis_buf = []
        self.rf_buf  = []

        # Fusion parameters
        self.spatial_thresh = 2.0    # meters
        self.time_window    = 0.1    # seconds
        self.rssi_thresh    = 0.5    # minimum RSSI
        self.required_class = 1      # expected visual class_id

        # Prepare CSV logging (with latency column)
        share_dir = get_package_share_directory('sensor_emulator')
        log_dir   = os.path.join(share_dir, 'logs')
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, 'detections.csv')
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['cue_time_s', 'x', 'y', 'fusion_mask', 'latency_s'])

        self.get_logger().info(f"Initialized CueFusionNode, logging to {self.csv_path}")

    def vis_cb(self, msg: String):
        try:
            cue = json.loads(msg.data)
            # record when ROS received this message
            recv = self.get_clock().now().nanoseconds * 1e-9
            cue['recv_ts'] = recv
            self.vis_buf.append(cue)
            self.try_fuse()
        except json.JSONDecodeError:
            self.get_logger().warning('Failed to parse visual cue JSON')

    def rf_cb(self, msg: String):
        try:
            cue = json.loads(msg.data)
            recv = self.get_clock().now().nanoseconds * 1e-9
            cue['recv_ts'] = recv
            self.rf_buf.append(cue)
            self.try_fuse()
        except json.JSONDecodeError:
            self.get_logger().warning('Failed to parse RF cue JSON')

    def try_fuse(self):
        # Prune old cues based on their own timestamp t
        all_ts = [v['t'] for v in self.vis_buf] + [r['t'] for r in self.rf_buf]
        if all_ts:
            max_t = max(all_ts)
            cutoff = max_t - 2 * self.time_window
            self.vis_buf = [v for v in self.vis_buf if v['t'] >= cutoff]
            self.rf_buf  = [r for r in self.rf_buf if r['t'] >= cutoff]

        # Look for any matching visual/rf pair
        for v in list(self.vis_buf):
            for r in list(self.rf_buf):
                dt   = abs(v['t'] - r['t'])
                dist = math.hypot(v['x'] - r['x'], v['y'] - r['y'])
                ok_class = (v.get('class_id') == self.required_class)
                ok_rssi  = (r.get('rssi', 0.0) >= self.rssi_thresh)

                if dt <= self.time_window and dist <= self.spatial_thresh and ok_class and ok_rssi:
                    # Compute fused position
                    fused_x = (v['x'] + r['x']) / 2.0
                    fused_y = (v['y'] + r['y']) / 2.0
                    fusion_mask = 0b11

                    # Determine fusion publish time & latency
                    fuse_time = self.get_clock().now().nanoseconds * 1e-9
                    start_ts  = max(v['recv_ts'], r['recv_ts'])
                    latency   = fuse_time - start_ts

                    # Build and publish fused message
                    out = {
                        't': max(v['t'], r['t']),
                        'x': fused_x,
                        'y': fused_y,
                        'fusion_mask': fusion_mask
                    }
                    self.fuse_pub.publish(String(data=json.dumps(out)))

                    # Append full row to CSV (with latency)
                    with open(self.csv_path, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([out['t'], fused_x, fused_y, fusion_mask, f"{latency:.6f}"])

                    # Log to screen
                    self.get_logger().info(
                        f"Fused at t={out['t']:.3f}s  → "
                        f"latency={latency*1000:.1f} ms  "
                        f"dt={dt:.3f}s  dist={dist:.2f}m"
                    )

                    # Remove used cues
                    self.vis_buf.remove(v)
                    self.rf_buf.remove(r)
                    return  # only fuse one pair per invocation

    def destroy_node(self):
        self.get_logger().info("Shutting down CueFusionNode")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CueFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

