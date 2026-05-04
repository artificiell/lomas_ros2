#!/usr/bin/env python3
import csv
import os
import threading
from collections import defaultdict
from datetime import datetime, timezone
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class DataCollection(Node):
    def __init__(self):
        super().__init__('data_collection_node')

        self._topics = [
            '/lomas/air/humidity',
            '/lomas/air/pressure',
            '/lomas/air/temperature',
            '/lomas/soil/moisture/left',
            '/lomas/soil/moisture/right',
        ]

        self._output_dir = os.path.expanduser('~/data/sensors/')
        os.makedirs(self._output_dir, exist_ok=True)

        self._buffers: dict[str, list[tuple[datetime, float]]] = defaultdict(list)
        self._hourly_averages: dict[str, list[tuple[str, float]]] = defaultdict(list)
        self._current_hour: int = datetime.now(timezone.utc).hour
        self._lock = threading.Lock()

        self._subscribers = []
        for topic in self._topics:
            sub = self.create_subscription(
                Float32,
                topic,
                self._make_callback(topic),
                qos_profile=10,
            )
            self._subscribers.append(sub)
            self.get_logger().info(f"Subscribed to topic: {topic}")

        self.create_timer(60.0, self._check_hour_rollover)
        self.get_logger().info("CollectionNode started. Aggregating hourly averages.")

    # ---------------------------------------------------------------------------
    # Callbacks & rollover
    # ---------------------------------------------------------------------------

    def _make_callback(self, topic: str):
        """Return a closure that captures the topic name."""
        def callback(msg: Float32):
            self._on_message(topic, msg.data)
        return callback

    def _on_message(self, topic: str, value: float):
        """Store an incoming sample with its CET timestamp; check for hour rollover."""
        now = datetime.now(ZoneInfo('Europe/Stockholm'))
        with self._lock:
            self._buffers[topic].append((now, value))
            self.get_logger().debug(
                f"[{topic}] received {value:.4f} at {now.isoformat()}"
            )
            if now.hour != self._current_hour:
                self._flush_hour(self._current_hour)
                self._current_hour = now.hour

    def _check_hour_rollover(self):
        """60-second safety-net: flush if the hour has changed between message arrivals."""
        now = datetime.now(timezone.utc)
        with self._lock:
            if now.hour != self._current_hour:
                self._flush_hour(self._current_hour)
                self._current_hour = now.hour

    # ---------------------------------------------------------------------------
    # Aggregation & persistence
    # ---------------------------------------------------------------------------

    def _flush_hour(self, previous_hour: int):
        """
        Compute per-topic averages for the completed hour, log them, and persist
        them to a date-stamped CSV file. Must be called while the lock is held.
        """
        now_utc = datetime.now(ZoneInfo('Europe/Stockholm'))
        date_str = now_utc.strftime('%Y-%m-%d')
        hour_label = f"{date_str} {previous_hour:02d}:00 CET"

        self.get_logger().info(f"--- Hourly averages for {hour_label} ---")

        rows: list[dict] = []
        for topic in self._topics:
            samples = self._buffers[topic]
            if samples:
                avg = sum(v for _, v in samples) / len(samples)
                n = len(samples)
                self.get_logger().info(
                    f"  {topic}: avg = {avg:.4f}  (n={n} samples)"
                )
            else:
                avg = None
                n = 0
                self.get_logger().warning(
                    f"  {topic}: no samples received this hour."
                )

            self._hourly_averages[topic].append((hour_label, avg))
            self._buffers[topic] = []

            rows.append({
                'date':    date_str,
                'hour':    f"{previous_hour:02d}:00",
                'topic':   topic,
                'average': f"{avg:.6f}" if avg is not None else '',
                'samples': n,
            })

        self._write_csv(date_str, rows)

    def _write_csv(self, date_str: str, rows: list[dict]):
        """
        Append hourly-average rows to ~/data/sensors/<date_str>.csv.
        The header is written only when the file does not yet exist.
        """
        filepath = os.path.join(self._output_dir, f"{date_str}.csv")
        file_exists = os.path.isfile(filepath)
        fieldnames = ['date', 'hour', 'topic', 'average', 'samples']

        try:
            with open(filepath, 'a', newline='', encoding='utf-8') as fh:
                writer = csv.DictWriter(fh, fieldnames=fieldnames)
                if not file_exists:
                    writer.writeheader()
                writer.writerows(rows)
            self.get_logger().info(f"Results appended to {filepath}")
        except OSError as exc:
            self.get_logger().error(f"Failed to write CSV ({filepath}): {exc}")

    # ---------------------------------------------------------------------------
    # Public API
    # ---------------------------------------------------------------------------

    def get_hourly_averages(self) -> dict[str, list[tuple[str, float]]]:
        """Return a thread-safe copy of all stored hourly averages."""
        with self._lock:
            return {
                topic: list(entries)
                for topic, entries in self._hourly_averages.items()
            }


# Main function
def main(args=None):
    rclpy.init(args=args)
    node = DataCollection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down data collection node... .")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
