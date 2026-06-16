#!/usr/bin/env python3
"""
lidar_detector.py
-----------------
ROS2 Humble node for the Unitree Go2 robot LiDAR obstacle detector.

Designed to work in NVIDIA Isaac Sim first, then on the physical robot.
The node subscribes to the Go2's point cloud topic, computes the nearest
obstacle distance, and prints one of three safety states:

  SAFE    — nearest obstacle > 2.0 m
  CAUTION — nearest obstacle between 1.0 m and 2.0 m
  STOP    — nearest obstacle < 1.0 m

Author : <Your Name>
Course : Computer Engineering – Robotics Research
Robot  : Unitree Go2  |  Sim: NVIDIA Isaac Sim
ROS    : ROS2 Humble  |  Python 3.10+
"""

# ─────────────────────────────────────────────
#  Standard-library imports
# ─────────────────────────────────────────────
import math                     # Used to compute 3-D Euclidean distance
import sys                      # Used for clean process exit on fatal errors

# ─────────────────────────────────────────────
#  ROS2 imports
# ─────────────────────────────────────────────
import rclpy                                        # Core ROS2 Python client library
from rclpy.node import Node                         # Base class for all ROS2 nodes
from sensor_msgs.msg import PointCloud2             # Standard ROS2 LiDAR message type
import sensor_msgs_py.point_cloud2 as pc2           # Helper to iterate over PointCloud2 data

# ─────────────────────────────────────────────
#  Safety-distance thresholds (metres)
#  Edit these constants to tune sensitivity.
# ─────────────────────────────────────────────
SAFE_DISTANCE    = 2.0   # Above this → SAFE
CAUTION_DISTANCE = 1.0   # Between CAUTION_DISTANCE and SAFE_DISTANCE → CAUTION
                         # Below CAUTION_DISTANCE → STOP

# ─────────────────────────────────────────────
#  ROS2 topic configuration
#  Isaac Sim publishes on the same topic that
#  the physical Go2 driver uses, so no code
#  change is needed when switching hardware.
# ─────────────────────────────────────────────
LIDAR_TOPIC = "/unitree_go2_0/lidar/point_cloud"


# ══════════════════════════════════════════════════════════════
#  LidarDetectorNode
# ══════════════════════════════════════════════════════════════
class LidarDetectorNode(Node):
    """
    ROS2 node that reads LiDAR point-cloud data and
    classifies the nearest obstacle into a safety state.
    """

    def __init__(self) -> None:
        """
        Constructor – called once when the node starts.

        Sets up:
          1. The ROS2 node name (visible in `ros2 node list`)
          2. The PointCloud2 subscription
          3. A message counter for basic diagnostics
        """
        # Initialise the base Node with a human-readable name
        super().__init__("lidar_detector_node")

        # ── Diagnostics counter ────────────────────────────────
        # Tracks how many point-cloud messages have been processed.
        # Useful for confirming data is arriving during simulation.
        self._msg_count: int = 0

        # ── Subscriber ────────────────────────────────────────
        # Subscribe to the Go2 LiDAR topic.
        # Queue depth of 10 is standard for sensor topics.
        # Every incoming message triggers self._on_point_cloud().
        self._subscription = self.create_subscription(
            PointCloud2,            # Message type
            LIDAR_TOPIC,            # Topic name
            self._on_point_cloud,   # Callback function
            10,                     # QoS queue depth
        )

        # Log startup information so the user can confirm the node is live
        self.get_logger().info(
            f"LidarDetectorNode started.\n"
            f"  Subscribed to : {LIDAR_TOPIC}\n"
            f"  SAFE  if dist > {SAFE_DISTANCE} m\n"
            f"  CAUTION if {CAUTION_DISTANCE} m <= dist <= {SAFE_DISTANCE} m\n"
            f"  STOP  if dist < {CAUTION_DISTANCE} m"
        )

    # ──────────────────────────────────────────────────────────
    #  Point-cloud callback
    # ──────────────────────────────────────────────────────────
    def _on_point_cloud(self, msg: PointCloud2) -> None:
        """
        Called automatically every time a PointCloud2 message
        arrives on LIDAR_TOPIC.

        Steps:
          1. Iterate over every 3-D point in the cloud.
          2. Compute the Euclidean distance from the robot's
             LiDAR origin to that point.
          3. Track the minimum (nearest) distance found.
          4. Classify and print the safety state.

        Parameters
        ----------
        msg : sensor_msgs.msg.PointCloud2
            The raw LiDAR data published by Isaac Sim or the
            physical Go2 driver.
        """
        self._msg_count += 1

        # ── Initialise nearest distance to a very large value ─
        # Any real obstacle will produce a smaller distance.
        nearest: float = float("inf")

        # ── Iterate over every point in the cloud ────────────
        # pc2.read_points() is a generator that yields named
        # tuples (x, y, z, intensity, …) for each LiDAR return.
        # skip_nans=True discards invalid returns (e.g. sky hits).
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point  # Unpack the 3-D coordinates

            # Euclidean distance from sensor origin to this point
            dist: float = math.sqrt(x**2 + y**2 + z**2)

            # Update the running minimum
            if dist < nearest:
                nearest = dist

        # ── Edge case: empty cloud ────────────────────────────
        # This can happen at startup or if the LiDAR returns no
        # valid data (e.g. pointing straight up in simulation).
        if nearest == float("inf"):
            self.get_logger().warn(
                f"[msg #{self._msg_count}] Point cloud is empty – no valid points received."
            )
            return

        # ── Classify and print the safety state ──────────────
        state: str = self._classify(nearest)
        print(f"[msg #{self._msg_count}] Nearest obstacle: {nearest:.3f} m  →  {state}")

    # ──────────────────────────────────────────────────────────
    #  Safety classifier
    # ──────────────────────────────────────────────────────────
    @staticmethod
    def _classify(distance: float) -> str:
        """
        Map a distance value to a safety state string.

        Parameters
        ----------
        distance : float
            Nearest obstacle distance in metres.

        Returns
        -------
        str
            One of: 'SAFE', 'CAUTION', 'STOP'
        """
        if distance > SAFE_DISTANCE:
            return "✅  SAFE"
        elif distance >= CAUTION_DISTANCE:
            return "⚠️  CAUTION"
        else:
            return "🛑  STOP"


# ══════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════
def main(args=None) -> None:
    """
    Standard ROS2 entry point.

    Lifecycle:
      1. rclpy.init()     – start the ROS2 middleware
      2. Node creation    – construct LidarDetectorNode
      3. rclpy.spin()     – block and process callbacks
      4. Cleanup          – destroy node and shut down rclpy
    """
    # ── Initialise ROS2 communication layer ──────────────────
    rclpy.init(args=args)

    # ── Create and spin the node ──────────────────────────────
    node = LidarDetectorNode()

    try:
        # spin() blocks until rclpy.shutdown() is called or
        # the process receives SIGINT (Ctrl+C)
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Graceful Ctrl+C handling – expected during development
        node.get_logger().info("Keyboard interrupt received. Shutting down.")

    finally:
        # ── Cleanup ───────────────────────────────────────────
        # Always destroy the node and shut down rclpy so that
        # middleware resources are released cleanly.
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


# ─────────────────────────────────────────────
#  Allow running as: python3 lidar_detector.py
# ─────────────────────────────────────────────
if __name__ == "__main__":
    main()
