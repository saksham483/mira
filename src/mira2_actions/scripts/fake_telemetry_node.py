#!/usr/bin/env python3
import sys
import traceback

# 1. Bulletproof Imports
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from custom_msgs.msg import Telemetry, Commands
    from vision_msgs.msg import Detection2DArray, Detection2D
except ImportError as e:
    print(f"\n[FATAL ERROR] Import failed: {e}")
    print("-> Did you forget to run 'source install/setup.bash' in this terminal tab?")
    print("-> Or are you missing 'vision_msgs'? (Run: sudo apt install ros-jazzy-vision-msgs)\n")
    sys.exit(1)

class FakeTelemetrySim(Node):
    def __init__(self):
        super().__init__('fake_telemetry_sim')

        self.current_depth = 0.0
        self.target_depth = 0.0
        self.current_mode = "IDLE"
        self.battery_voltage = 14.8

        self.get_logger().info("=== Fake Telemetry & Pixhawk Simulator Started ===")

        # Publishers
        self.telemetry_pub = self.create_publisher(Telemetry, '/master/telemetry', 10)
        self.vision_pub = self.create_publisher(Detection2DArray, '/vision/detections', 10)

        # Subscribers
        self.create_subscription(String, '/controller/state', self.state_callback, 10)
        self.create_subscription(Commands, '/master/commands', self.cmd_callback, 10)

        # 10Hz Simulation Loop
        self.timer = self.create_timer(0.1, self.sim_loop_safe)
        self.get_logger().info("Publishing /master/telemetry at 10Hz. Waiting for commands...")

    def state_callback(self, msg: String):
        data = msg.data.split(',')
        action = data[0]

        if action == "DIVE" and len(data) > 1:
            self.target_depth = float(data[1])
            self.get_logger().info(f"[SIM] Received DIVE command! Target depth: {self.target_depth}m")
        elif action == "IDLE":
            self.get_logger().info("[SIM] Controller switched to IDLE")

        self.current_mode = action

    def cmd_callback(self, msg: Commands):
        # Using hasattr to prevent crashes if your custom_msgs/Commands.msg is structured differently
        if hasattr(msg, 'mode') and msg.mode == "MANUAL" and hasattr(msg, 'thrust') and msg.thrust > 1600:
            self.target_depth = 0.0
            self.get_logger().info("[SIM] EMERGENCY COMMAND received. Surfacing!")

    def sim_loop_safe(self):
        try:
            self.sim_loop()
        except Exception as e:
            self.get_logger().error(f"Error in timer loop: {e}")
            self.get_logger().error(traceback.format_exc())

    def sim_loop(self):
        # 1. Simulate Depth Dynamics
        depth_error = self.target_depth - self.current_depth
        if abs(depth_error) > 0.01:
            step = 0.02 if depth_error > 0 else -0.05
            self.current_depth += step
            
            # Clamp to target depth
            if (step > 0 and self.current_depth > self.target_depth) or \
               (step < 0 and self.current_depth < self.target_depth):
                self.current_depth = self.target_depth

        # 2. Publish Mock Telemetry safely!
        telem = Telemetry()
        
        # type(telem.field) dynamically casts our float to an int if the .msg expects an int
        telem.external_pressure = type(telem.external_pressure)(self.current_depth)
        telem.battery_voltage = type(telem.battery_voltage)(self.battery_voltage)
        
        if hasattr(telem, 'imu_xacc'):  
            telem.imu_xacc = type(telem.imu_xacc)(0.0)
            telem.imu_yacc = type(telem.imu_yacc)(0.0)
        
        self.telemetry_pub.publish(telem)

        # 3. Publish Mock Vision Detection during SEARCH
        if "SEARCH" in self.current_mode or "TRACK" in self.current_mode:
            det_array = Detection2DArray()
            detection = Detection2D()
            
            if hasattr(detection, 'id'):
                detection.id = "gate"
            
            detection.bbox.center.position.x = 0.5
            detection.bbox.center.position.y = 0.5
            
            detection.bbox.size_x = 0.8
            detection.bbox.size_y = 0.8
            
            det_array.detections.append(detection)
            self.vision_pub.publish(det_array)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = FakeTelemetrySim()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[SIM] Shutting down Simulator...")
    except Exception as e:
        print(f"\n[FATAL ERROR] Node crashed: {e}")
        traceback.print_exc()
    finally:
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()