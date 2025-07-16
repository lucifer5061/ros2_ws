#!/usr/bin/env python3
import rclpy, random, math, json, os, yaml
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from sensor_emulator import motion_profiles as mp

class TestInputPublisher(Node):

    def __init__(self):
        super().__init__('test_input_pub')
        self.vis_pub = self.create_publisher(String, '/mock/visual_cue', 10)
        self.rf_pub  = self.create_publisher(String, '/mock/rf_cue',     10)
        self.dt = 0.1  # 10 Hz
        self.counter = 0
        self.match_every = 5.0
        self.last_match_t = -self.match_every

        # Load motion profile and mode
        self.motion_type, self.motion_cfg = self.load_motion_profile()
        self.mode = self.motion_cfg.get("input", {}).get("mode", "synthetic")
        self.get_logger().info(f'Mode: {self.mode} | Motion: {self.motion_type}')

        if self.mode == 'replay':
            file_path = self.motion_cfg['input']['file_path']
            self.replay_data = self.load_replay_data(file_path)
            self.replay_index = 0
            self.timer = self.create_timer(self.dt, self.replay_timer_cb)
        else:
            self.timer = self.create_timer(self.dt, self.publish_cues)

        self.get_logger().info('TestInputPublisher started')

    def load_motion_profile(self):
        pkg_dir = get_package_share_directory('sensor_emulator')
        cfg_path = os.path.join(pkg_dir, 'config', 'motion_profile.yaml')
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)
        motion_cfg = cfg['motion']
        motion_type = motion_cfg['type']
        return motion_type, motion_cfg

    def load_replay_data(self, file_path):
        full_path = os.path.join(get_package_share_directory('sensor_emulator'), file_path)
        with open(full_path, 'r') as f:
            data = json.load(f)
        data.sort(key=lambda x: x['t'])
        self.get_logger().info(f"Loaded {len(data)} cues from {file_path}")
        return data

    def get_position(self, t):
        if self.motion_type == 'linear':
            return mp.linear_motion(t, speed=self.motion_cfg.get("speed", 1.0))
        elif self.motion_type == 'zigzag':
            return mp.zigzag_motion(t, speed=self.motion_cfg.get("speed", 1.0),
                                    period=self.motion_cfg.get("period", 2.0))
        elif self.motion_type == 'circle':
            return mp.circle_motion(t, radius=self.motion_cfg.get("radius", 5.0),
                                    omega=self.motion_cfg.get("omega", 0.5))
        elif self.motion_type == 'hover':
            return mp.hover_drift(t, amp=self.motion_cfg.get("drift_amp", 0.5))
        else:
            self.get_logger().warn(f"Unknown motion profile: {self.motion_type}")
            return 0.0, 0.0, 0.0

    def publish_cues(self):
        t = self.counter * self.dt
        x, y, _ = self.get_position(t)

        if t - self.last_match_t >= self.match_every:
            visual = {'t': t, 'x': x, 'y': y, 'class_id': 1, 'confidence': 0.95}
            rf     = {'t': t + random.uniform(-0.02, 0.02),
                      'x': x + random.uniform(-0.2, 0.2),
                      'y': y + random.uniform(-0.2, 0.2),
                      'rssi': 0.9}
            self.vis_pub.publish(String(data=json.dumps(visual)))
            self.rf_pub.publish(String(data=json.dumps(rf)))
            self.get_logger().info(f'Published MATCH at t={t:.1f}')
            self.last_match_t = t
        else:
            self.vis_pub.publish(String(data=json.dumps({
                't': t,
                'x': random.uniform(0, 50),
                'y': random.uniform(0, 50),
                'class_id': 0,
                'confidence': random.uniform(0, 0.5)
            })))
            self.rf_pub.publish(String(data=json.dumps({
                't': t,
                'x': random.uniform(0, 50),
                'y': random.uniform(0, 50),
                'rssi': random.uniform(0, 0.4)
            })))
        self.counter += 1

    def replay_timer_cb(self):
        if self.replay_index >= len(self.replay_data):
            self.get_logger().info("Replay complete.")
            self.timer.cancel()
            return

        cue = self.replay_data[self.replay_index]
        t = self.counter * self.dt
        if cue['t'] <= t:
            msg = String(data=json.dumps(cue))
            if cue['type'] == 'visual':
                self.vis_pub.publish(msg)
            elif cue['type'] == 'rf':
                self.rf_pub.publish(msg)
            else:
                self.get_logger().warn(f"Unknown cue type: {cue['type']}")
            self.replay_index += 1
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestInputPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

