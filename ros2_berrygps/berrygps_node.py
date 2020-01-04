
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from ros2_berrygps.gps_node import GPSNode
from ros2_berrygps.imu_node import IMUNode


# TODO, turn into lifecycle nodes

def main():
    rclpy.init()

    try:
        gps = GPSNode()
        imu = IMUNode()

        executor = SingleThreadedExecutor()
        # Add imported nodes to this executor
        executor.add_node(gps)
        executor.add_node(imu)

        try:    
            executor.spin()
        finally:
            executor.shutdown()
            gps.destroy_node()
            imu.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
