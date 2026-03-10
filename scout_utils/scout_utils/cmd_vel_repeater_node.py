#!/usr/bin/env python3
"""
cmd_vel_repeater_node.py

Bridges the gap between nav stacks that follow the ROS2 "send once" convention
and robot drivers that require periodic velocity commands (hardware watchdog).

Subscribes to an input cmd_vel topic, stores the latest Twist or TwistStamped,
and republishes it at a fixed frequency to the driver's topic. If no new command
has arrived within `input_timeout_sec`, a zero Twist is published to safely halt
the robot.

Parameters:
    input_topic         (str,   default: 'cmd_vel')        Source topic.
    output_topic        (str,   default: 'cmd_vel_driver') Driver topic.
    publish_rate_hz     (float, default: 10.0)             Repeat frequency.
    input_timeout_sec   (float, default: 0.5)              Safety zero timeout.
    use_stamped_cmd_vel (bool,  default: False)            Use TwistStamped.
"""

import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header


class CmdVelRepeater(Node):
    def __init__(self):
        super().__init__("cmd_vel_repeater")

        # --- Parameters ---
        self.declare_parameter("input_topic", "cmd_vel")
        self.declare_parameter("output_topic", "cmd_vel_driver")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("input_timeout_sec", 0.5)
        # When True, both the subscription and publisher use TwistStamped.
        # Nav2 Jazzy uses TwistStamped by default; older stacks use Twist.
        self.declare_parameter("use_stamped_cmd_vel", False)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self._timeout = self.get_parameter("input_timeout_sec").value
        self._stamped = self.get_parameter("use_stamped_cmd_vel").value

        # --- State ---
        # Internally we always cache a plain Twist for simplicity.
        # The stamp is regenerated fresh on every publish so it reflects
        # actual send time, not when the command was originally received.
        self._latest_twist = Twist()
        self._last_received_time = None  # None = never received anything
        self._timed_out = False  # suppresses repeated timeout warnings
        self._lock = threading.Lock()

        # --- ROS interfaces ---
        if self._stamped:
            self._sub = self.create_subscription(
                TwistStamped, input_topic, self._stamped_callback, 10
            )
            self._pub = self.create_publisher(TwistStamped, output_topic, 10)
        else:
            self._sub = self.create_subscription(
                Twist, input_topic, self._twist_callback, 10
            )
            self._pub = self.create_publisher(Twist, output_topic, 10)

        period = 1.0 / publish_rate_hz
        self._timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f"cmd_vel_repeater started:\n"
            f'  type   : {"TwistStamped" if self._stamped else "Twist"}\n'
            f"  input  : {input_topic}\n"
            f"  output : {output_topic}\n"
            f"  rate   : {publish_rate_hz} Hz\n"
            f"  timeout: {self._timeout} s "
            f'({"disabled" if self._timeout <= 0.0 else "active"})'
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _twist_callback(self, msg: Twist) -> None:
        with self._lock:
            self._latest_twist = msg
            self._last_received_time = self.get_clock().now()
            self._timed_out = False

    def _stamped_callback(self, msg: TwistStamped) -> None:
        # Unwrap to plain Twist so the rest of the logic stays uniform.
        with self._lock:
            self._latest_twist = msg.twist
            self._last_received_time = self.get_clock().now()
            self._timed_out = False

    # ------------------------------------------------------------------
    # Timer callback — runs at publish_rate_hz
    # ------------------------------------------------------------------

    def _timer_callback(self) -> None:
        with self._lock:
            # Never received anything yet — stay silent.
            if self._last_received_time is None:
                return

            if self._timeout > 0.0:
                age = (
                    self.get_clock().now() - self._last_received_time
                ).nanoseconds * 1e-9

                if age > self._timeout:
                    if not self._timed_out:
                        self.get_logger().warn(
                            f"No cmd_vel received for {age:.2f} s "
                            f"(threshold: {self._timeout} s). "
                            f"Publishing zero Twist for safety."
                        )
                        self._timed_out = True
                    self._publish(Twist())
                    return

            self._publish(self._latest_twist)

    # ------------------------------------------------------------------
    # Publish helper — wraps in TwistStamped if needed
    # ------------------------------------------------------------------

    def _publish(self, twist: Twist) -> None:
        if self._stamped:
            msg = TwistStamped()
            # Use current time as stamp — reflects actual send time.
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist = twist
            self._pub.publish(msg)
        else:
            self._pub.publish(twist)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRepeater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
