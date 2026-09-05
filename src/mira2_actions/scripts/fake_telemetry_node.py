#!/usr/bin/env python3
import sys
import traceback

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from custom_msgs.msg import Telemetry, Commands
    from vision_msgs.msg import Detection2DArray, Detection2D
except ImportError as e:
    print(f"\n[FATAL ERROR] Import failed: {e}")
    sys.exit(1)

class FullMissionSimulator(Node):
    def __init__(self):
        super().__init__('full_mission_simulator')

        self.current_depth = 0.0
        self.target_depth = 0.0
        self.current_mode = "IDLE"

        # Track bounding box sizes to simulate moving closer to targets
        self.vision_sizes = {} 

        self.get_logger().info("=== GOD MODE SIMULATOR STARTED ===")
        self.get_logger().info("Faking all telemetry and perfecting all vision targets.")

        self.telemetry_pub = self.create_publisher(Telemetry, '/master/telemetry', 10)
        self.vision_pub = self.create_publisher(Detection2DArray, '/vision/detections', 10)

        self.create_subscription(String, '/controller/state', self.state_callback, 10)
        self.create_subscription(Commands, '/master/commands', self.cmd_callback, 10)

        self.timer = self.create_timer(0.1, self.sim_loop_safe)

    def state_callback(self, msg: String):
        data = msg.data.split(',')
        action = data[0]

        if action != self.current_mode:
            self.get_logger().info(f"[SIM] BT State Changed to: {action}")

        if action == "DIVE" and len(data) > 1:
            self.target_depth = float(data[1])
        elif action == "SURFACE":
            self.target_depth = 0.0
        
        self.current_mode = action

    def cmd_callback(self, msg: Commands):
        # Catch emergency abort commands sent by the Behavior Tree
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
        # --- 1. SIMULATE SMOOTH DIVING ---
        depth_error = self.target_depth - self.current_depth
        if abs(depth_error) > 0.01:
            self.current_depth += 0.05 if depth_error > 0 else -0.05
            # Snap to target if very close to prevent jitter
            if abs(self.target_depth - self.current_depth) < 0.06:
                self.current_depth = self.target_depth

        # --- 2. PUBLISH PERFECT TELEMETRY ---
        telem = Telemetry()
        telem.external_pressure = type(telem.external_pressure)(self.current_depth)
        telem.battery_voltage = type(telem.battery_voltage)(15.5)  # Perfect battery health
        
        if hasattr(telem, 'imu_xacc'):  
            telem.imu_xacc = type(telem.imu_xacc)(0.0)
            telem.imu_yacc = type(telem.imu_yacc)(0.0)
        
        self.telemetry_pub.publish(telem)

        # --- 3. GOD MODE VISION TARGETS ---
        # We publish every standard mission target perfectly centered (0.5, 0.5).
        # When in SEARCH or TRACK, we slowly increase the bounding box size 
        # to simulate the AUV moving closer to the target until it reaches 0.9.
        
        det_array = Detection2DArray()
        
        # Add any other target names your XML file looks for here!
        targets = ["gate", "flare", "mat", "bucket", "drum", "red_flare", "yellow_flare"]
        
        for t in targets:
            if t not in self.vision_sizes:
                self.vision_sizes[t] = 0.1
            
            if self.current_mode in ["SEARCH", "TRACK"]:
                if self.vision_sizes[t] < 0.90:
                    self.vision_sizes[t] += 0.02 # Simulate approach speed
            else:
                self.vision_sizes[t] = 0.1 # Reset size if idle/diving

            detection = Detection2D()
            if hasattr(detection, 'id'):
                detection.id = t
            
            # Perfectly centered for your setup
            detection.bbox.center.position.x = 0.5
            detection.bbox.center.position.y = 0.5
            detection.bbox.size_x = float(self.vision_sizes[t])
            detection.bbox.size_y = float(self.vision_sizes[t])
            
            det_array.detections.append(detection)

        self.vision_pub.publish(det_array)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = FullMissionSimulator()
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
