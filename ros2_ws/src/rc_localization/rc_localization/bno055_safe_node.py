#!/usr/bin/env python3

# Lifecycle wrapper derived from flynneva/bno055 bno055.py (BSD-3-Clause).
# Sensor transport, register handling and publishers remain in the pinned
# upstream package; only process shutdown ownership is changed here.

import threading

from bno055.bno055 import Bno055Node
from bno055.error_handling.exceptions import BusOverRunException
import rclpy
from rclpy.executors import ExternalShutdownException


def main(args=None):
    rclpy.init(args=args)
    node = None
    timers = []
    try:
        node = Bno055Node()
        node.setup()
        lock = threading.Lock()

        def read_data():
            if not lock.acquire(blocking=False):
                node.get_logger().warning(
                    "BNO055 query still running; skipping this cycle",
                    throttle_duration_sec=1.0,
                )
                return
            try:
                node.sensor.get_sensor_data()
            except (BusOverRunException, ZeroDivisionError):
                pass
            except Exception as error:  # noqa: B902
                node.get_logger().warning(
                    f"Receiving BNO055 data failed: "
                    f"{type(error).__name__}: {error}",
                    throttle_duration_sec=1.0,
                )
            finally:
                lock.release()

        def publish_calibration_status():
            if not lock.acquire(blocking=False):
                return
            try:
                node.sensor.get_calib_status()
            except Exception as error:  # noqa: B902
                node.get_logger().warning(
                    f"Receiving BNO055 calibration failed: "
                    f"{type(error).__name__}: {error}",
                    throttle_duration_sec=5.0,
                )
            finally:
                lock.release()

        data_frequency = float(node.param.data_query_frequency.value)
        calibration_frequency = float(
            node.param.calib_status_frequency.value
        )
        if data_frequency <= 0.0 or calibration_frequency <= 0.0:
            raise ValueError("BNO055 timer frequencies must be positive")
        timers = [
            node.create_timer(1.0 / data_frequency, read_data),
            node.create_timer(
                1.0 / calibration_frequency,
                publish_calibration_status,
            ),
        ]
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            for timer in timers:
                node.destroy_timer(timer)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

