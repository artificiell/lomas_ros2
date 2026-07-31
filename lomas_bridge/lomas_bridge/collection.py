#!/usr/bin/env python3

import json
import os
from datetime import datetime
from zoneinfo import ZoneInfo

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Bool, String

from cam_interfaces.srv import Image
from lomas_interfaces.msg import MachineCommand, MachineStatus


class CollectionScan(Node):

    STEP = 1.0  # Machine units advanced per timer tick, once the previous step is confirmed

    def __init__(self):
        super().__init__('collection_scan_node')

        # Parameters
        self.declare_parameter('machine_max_x',      24.0)
        self.declare_parameter('machine_max_y',      24.0)
        self.declare_parameter('service_name',       'camera/image')
        self.declare_parameter('x_start',            2.0)
        self.declare_parameter('y_start',            2.0)
        self.declare_parameter('x_step',             4.0)
        self.declare_parameter('y_step',             5.0)
        self.declare_parameter('position_tolerance', 0.1)   # how close to a target counts as "reached"
        self.declare_parameter('init_delay_sec',     5.0)   # wait after stop before scan starts
        self.declare_parameter('settle_delay_sec',   2.0)   # wait after arriving at a normal waypoint
        self.declare_parameter('column_delay_sec',   10.0)  # wait after arriving at first wp of new column
        self.declare_parameter('end_delay_sec',      5.0)   # wait after arriving at the last waypoint
        self.declare_parameter('quality',            80)    # JPEG quality 1-99 to request compression; 0 (or >=100) = raw image
        self.declare_parameter('max_grab_retries',   1)     # retries for a failed image grab before giving up on a waypoint
        self.declare_parameter('grab_retry_delay_sec', 2.0) # wait before retrying a failed grab
        self.declare_parameter('grab_timeout_sec',   10.0)  # treat a grab as failed if no response by then

        self._max_x        = max(1.0, float(self.get_parameter('machine_max_x').value))
        self._max_y        = max(1.0, float(self.get_parameter('machine_max_y').value))
        self._x_start       = float(self.get_parameter('x_start').value)
        self._y_start       = float(self.get_parameter('y_start').value)
        self._x_step        = float(self.get_parameter('x_step').value)
        self._y_step        = float(self.get_parameter('y_step').value)
        self._tolerance     = float(self.get_parameter('position_tolerance').value)
        self._init_delay    = float(self.get_parameter('init_delay_sec').value)
        self._settle_delay  = float(self.get_parameter('settle_delay_sec').value)
        self._column_delay  = float(self.get_parameter('column_delay_sec').value)
        self._end_delay     = float(self.get_parameter('end_delay_sec').value)
        self._service_name  = self.get_parameter('service_name').get_parameter_value().string_value
        self._quality       = int(self.get_parameter('quality').value)
        self._max_grab_retries    = int(self.get_parameter('max_grab_retries').value)
        self._grab_retry_delay    = float(self.get_parameter('grab_retry_delay_sec').value)
        self._grab_timeout        = float(self.get_parameter('grab_timeout_sec').value)

        # Image grabbing
        self._tz         = ZoneInfo('Europe/Stockholm')
        self._output_dir = os.path.expanduser('~/data/images/')
        self._bridge     = CvBridge()
        self._date_str   = ''

        os.makedirs(self._output_dir, exist_ok=True)

        # Camera service client
        self._cam_client = self.create_client(Image, self._service_name)
        self.get_logger().info(f"Waiting for camera service '{self._service_name}' ...")
        while rclpy.ok() and not self._cam_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"  '{self._service_name}' not available yet ...")
        self.get_logger().info(f"Connected to camera service: '{self._service_name}'")

        # ROS publishers & subscribers
        self._machine_cmd_pub = self.create_publisher(MachineCommand, 'lomas/machine/cmd', 10)
        self._web_state_pub   = self.create_publisher(String, 'lomas/control/state', 10)

        self.create_subscription(Bool, 'lomas/machine/stop', self._stop_callback, 10)
        self.create_subscription(MachineStatus, 'lomas/machine/status', self._machine_status_callback, 10)

        # Motion state
        self._x_current = 0.0
        self._y_current = 0.0
        self._x_target  = None
        self._y_target  = None

        # Real machine feedback, and whether we're waiting on it to confirm the
        # last commanded step before the scan is allowed to proceed further
        self._machine_status         = None
        self._awaiting_confirmation  = False
        self._last_commanded_x       = None
        self._last_commanded_y       = None

        # Scan state
        self._waypoints:     list[tuple[float, float]] = []
        self._column_starts: set[int]                  = set()  # wp indices that start a new column
        self._wp_index:      int  = 0
        self._running:       bool = False
        self._grabbing:      bool = False
        self._settling:      bool = False
        self._settle_timer        = None

        # Image-grab retry/timeout state
        self._grab_retry_count  = 0
        self._retry_timer       = None
        self._grab_timeout_timer = None
        self._grab_completed    = True  # no grab in flight yet

        # Main scan timer (10 Hz)
        self._scan_timer = self.create_timer(0.1, self._timer_callback)

        self._initialise()

    # ---------------------------------------------------------------------------
    # Initialisation
    # ---------------------------------------------------------------------------

    def _initialise(self) -> None:
        self.get_logger().info(
            f"Publishing stop command. Scan will begin in {self._init_delay:.1f} s."
        )
        self._publish_cmd_and_state(0.0, 0.0, stop=True)
        self._init_timer = self.create_timer(self._init_delay, self._start_scan)

    def _start_scan(self) -> None:
        self._init_timer.cancel()
        self._init_timer = None

        self._date_str = datetime.now(self._tz).strftime('%Y-%m-%d')
        scan_folder    = os.path.join(self._output_dir, self._date_str)
        os.makedirs(scan_folder, exist_ok=True)

        self._waypoints, self._column_starts = self._build_waypoints()
        self._wp_index = 0
        self._running  = True
        self._grabbing = False
        self._settling = False
        self._set_next_target()

        self.get_logger().info(
            f"Scan started: {len(self._waypoints)} waypoints across "
            f"{len(self._column_starts)} columns. Images → {scan_folder}"
        )

    # ---------------------------------------------------------------------------
    # Waypoint generation
    # ---------------------------------------------------------------------------

    def _build_waypoints(self) -> tuple[list[tuple[float, float]], set[int]]:
        """
        Column-major scan: sweep Y for each fixed X, then step to the next X.
        Also returns the set of waypoint indices that START a new column (index > 0),
        so the longer column_delay is applied after the machine arrives there.

        Example with x_start=2, y_start=2, x_step=4, y_step=4, max=(24,24):
          col x=2:  (2,2),  (2,6),  (2,10), (2,14), (2,18), (2,22)
          col x=6:  (6,2),* (6,6),  (6,10), ...       * = column_start index
          col x=10: (10,2), ...
        """
        waypoints      = []
        column_starts  = set()
        x = self._x_start
        while x <= self._max_x:
            if waypoints:                           # every column after the first
                column_starts.add(len(waypoints))   # index of the first wp in this column
            y = self._y_start
            while y <= self._max_y:
                waypoints.append((x, y))
                y += self._y_step
            x += self._x_step
        return waypoints, column_starts

    # ---------------------------------------------------------------------------
    # ROS subscribers
    # ---------------------------------------------------------------------------

    def _machine_status_callback(self, msg: MachineStatus) -> None:
        self._machine_status = msg

    def _stop_callback(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info("Stop received — aborting scan.")
            self._running  = False
            self._grabbing = False
            self._settling = False
            if self._settle_timer is not None:
                self._settle_timer.cancel()
                self._settle_timer = None
            if self._retry_timer is not None:
                self._retry_timer.cancel()
                self._retry_timer = None
            if self._grab_timeout_timer is not None:
                self._grab_timeout_timer.cancel()
                self._grab_timeout_timer = None
            self._grab_completed    = True
            self._grab_retry_count = 0
            self._x_current, self._y_current = 0.0, 0.0
            self._x_target, self._y_target   = None, None
            self._publish_cmd_and_state(0.0, 0.0, stop=True)

    # ---------------------------------------------------------------------------
    # Machine feedback
    # ---------------------------------------------------------------------------

    def _step_confirmed(self) -> bool:
        """True once /machine/status reports the machine idle and within
        tolerance of the last commanded x/y — actual hardware confirmation
        rather than assuming a step succeeded once it's been published."""
        if self._machine_status is None or self._last_commanded_x is None:
            return False
        if self._machine_status.machine_moving:
            return False
        dx = abs(self._machine_status.x - self._last_commanded_x)
        dy = abs(self._machine_status.y - self._last_commanded_y)
        return dx <= self._tolerance and dy <= self._tolerance

    # ---------------------------------------------------------------------------
    # Timer: motion sequencing
    # ---------------------------------------------------------------------------

    def _timer_callback(self) -> None:
        if not self._running or self._x_target is None or self._grabbing or self._settling:
            return

        # Don't take (or credit) another step until the machine has actually
        # confirmed the last one — previously this assumed a step succeeded
        # the instant it was published, with no hardware feedback at all.
        if self._awaiting_confirmation:
            if not self._step_confirmed():
                return
            self._awaiting_confirmation = False

            # If the confirmed step completed the waypoint, move to settling now
            if self._x_current == self._x_target and self._y_current == self._y_target:
                self._begin_settle()
            return

        # Column-major movement: resolve Y first (within column), then X (between columns).
        # Steps are clamped so they can't overshoot a fractional target.
        if self._y_current < self._y_target:
            self._y_current = min(self._y_current + self.STEP, self._y_target)
        elif self._y_current > self._y_target:
            self._y_current = max(self._y_current - self.STEP, self._y_target)
        elif self._x_current < self._x_target:
            self._x_current = min(self._x_current + self.STEP, self._x_target)
        elif self._x_current > self._x_target:
            self._x_current = max(self._x_current - self.STEP, self._x_target)

        self._publish_cmd_and_state(self._x_current, self._y_current, stop=False)

    def _begin_settle(self) -> None:
        """Waypoint arrival has been confirmed by real machine feedback —
        pick the right delay and wait before grabbing an image."""
        is_scan_end  = self._wp_index == len(self._waypoints) - 1
        is_col_start = self._wp_index in self._column_starts

        if is_scan_end:
            delay       = self._end_delay
            delay_label = f"end delay {delay:.1f} s"
        elif is_col_start:
            delay       = self._column_delay
            delay_label = f"column-start delay {delay:.1f} s (long X transit just completed)"
        else:
            delay       = self._settle_delay
            delay_label = f"settle delay {delay:.1f} s"

        self.get_logger().info(
            f"Arrived at waypoint {self._wp_index + 1}/{len(self._waypoints)}: "
            f"({self._x_current}, {self._y_current}) — waiting {delay_label}"
        )
        self._settling     = True
        self._settle_timer = self.create_timer(delay, self._on_settled)

    def _on_settled(self) -> None:
        """Fires after the settle/column/end delay — kick off the image grab."""
        self._settle_timer.cancel()
        self._settle_timer = None
        self._settling     = False
        self._grabbing     = True
        self._grab_retry_count = 0
        self._grab_image()

    # ---------------------------------------------------------------------------
    # Image grabbing
    # ---------------------------------------------------------------------------

    def _grab_image(self) -> None:
        self._grab_completed = False
        req         = Image.Request()
        req.quality = self._quality
        future      = self._cam_client.call_async(req)
        future.add_done_callback(self._handle_image_response)

        # Guard against a request that never completes (e.g. the camera
        # service is restarting) — without this, a lost response would
        # stall the whole scan forever at this waypoint.
        self._grab_timeout_timer = self.create_timer(self._grab_timeout, self._on_grab_timeout)

    def _on_grab_timeout(self) -> None:
        self._grab_timeout_timer.cancel()
        self._grab_timeout_timer = None
        if self._grab_completed:
            return  # real response already arrived and was handled
        self._grab_completed = True
        self.get_logger().error(
            f"Image grab timed out after {self._grab_timeout:.1f}s (camera service may be restarting)."
        )
        self._handle_grab_failure()

    def _decode_response_image(self, response):
        """Decode whichever field the server actually populated."""
        if response.compressed.data:
            return self._bridge.compressed_imgmsg_to_cv2(response.compressed, desired_encoding='bgr8')
        return self._bridge.imgmsg_to_cv2(response.image, desired_encoding='bgr8')

    def _handle_image_response(self, future) -> None:
        if self._grab_completed:
            return  # the timeout already fired and handled this attempt
        self._grab_completed = True
        if self._grab_timeout_timer is not None:
            self._grab_timeout_timer.cancel()
            self._grab_timeout_timer = None

        success = False
        try:
            response = future.result()
            if response is None:
                self.get_logger().error("Camera service returned no response.")
            elif not response.success:
                self.get_logger().error("Camera service reported a failed capture.")
            else:
                img      = self._decode_response_image(response)
                filepath = self._build_filepath()
                cv2.imwrite(filepath, img)
                self.get_logger().info(f"Image saved: {filepath}")
                success = True

        except Exception as e:
            self.get_logger().error(f"Image grab failed: {e}")

        if success:
            self._grab_retry_count = 0
            self._grabbing = False
            self._advance_waypoint()
        else:
            self._handle_grab_failure()

    def _handle_grab_failure(self) -> None:
        # Retry a failed grab a bounded number of times before giving up on
        # this waypoint entirely — rather than either hanging indefinitely
        # or silently skipping on the very first failure.
        if self._grab_retry_count < self._max_grab_retries:
            self._grab_retry_count += 1
            self.get_logger().warn(
                f"Retrying image grab ({self._grab_retry_count}/{self._max_grab_retries}) "
                f"in {self._grab_retry_delay:.1f}s..."
            )
            self._retry_timer = self.create_timer(self._grab_retry_delay, self._retry_grab)
        else:
            retries_label = "retry" if self._max_grab_retries == 1 else "retries"
            self.get_logger().error(
                f"Giving up on this waypoint after {self._max_grab_retries} {retries_label} "
                f"— moving on without an image."
            )
            self._grab_retry_count = 0
            self._grabbing = False
            self._advance_waypoint()

    def _retry_grab(self) -> None:
        self._retry_timer.cancel()
        self._retry_timer = None
        self._grab_image()

    def _build_filepath(self) -> str:
        timestamp = datetime.now(self._tz).strftime('%Y-%m-%d_%H-%M-%S')
        sanitized = self._service_name.strip('/').replace('/', '_')
        # x_current/y_current are float32-precision now; the filename only needs
        # a readable label, so it's rounded to the nearest whole unit here —
        # this does NOT affect the precision of any commanded machine position.
        x_label   = round(self._x_current)
        y_label   = round(self._y_current)
        filename  = f"{sanitized}_x{x_label:03d}_y{y_label:03d}_{timestamp}.png"
        return os.path.join(self._output_dir, self._date_str, filename)

    # ---------------------------------------------------------------------------
    # Waypoint management
    # ---------------------------------------------------------------------------

    def _set_next_target(self) -> None:
        x, y = self._waypoints[self._wp_index]
        self._x_target, self._y_target = x, y
        self.get_logger().info(
            f"Moving to waypoint {self._wp_index + 1}/{len(self._waypoints)}: ({x}, {y})"
        )

    def _advance_waypoint(self) -> None:
        is_scan_end = self._wp_index == len(self._waypoints) - 1
        self._wp_index += 1

        if is_scan_end:
            self.get_logger().info("Scan complete. Returning to origin (0, 0).")
            self._running  = False
            self._x_target = 0.0
            self._y_target = 0.0
            self._publish_cmd_and_state(0.0, 0.0, stop=True)
        else:
            self._set_next_target()

    # ---------------------------------------------------------------------------
    # Publishing
    # ---------------------------------------------------------------------------

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def _publish_cmd_and_state(self, x: float, y: float, stop: bool = False) -> None:
        cmd     = MachineCommand()
        cmd.x   = float(x)
        cmd.y   = float(y)
        cmd.z   = 0.0  # this testbed only moves in the x/y plane
        cmd.stop = stop
        self._machine_cmd_pub.publish(cmd)

        # Track what was last asked of the machine, so it can be confirmed
        # against real /machine/status feedback in _step_confirmed()
        self._last_commanded_x      = float(x)
        self._last_commanded_y      = float(y)
        self._awaiting_confirmation = True

        x_norm     = self._clamp(x / self._max_x, 0.0, 1.0)
        y_norm     = self._clamp(y / self._max_y, 0.0, 1.0)
        state      = String()
        state.data = json.dumps({"y": round(x_norm, 4), "x": round(y_norm, 4)})
        self._web_state_pub.publish(state)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CollectionScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
