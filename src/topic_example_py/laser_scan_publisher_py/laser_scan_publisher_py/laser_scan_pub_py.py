import rclpy
from rclpy.node import Node
from sensor_msgs.msg._laser_scan import LaserScan
import time

class LaserScanPublisherSim(Node):

    def __init__(self):
        super().__init__('LaserScanPublisherSim')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        timer_period = 0.03  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.DEG2RAD = 3.14/180.0
        
        self.laser_msgs_= LaserScan()
        self.laser_msgs_._header.frame_id = "laser_frame_link"

        angle_resolution = 2500
        start_angle = -450000
        stop_angle = 2250000
        scan_frequency = 2500
        
        angle_range = stop_angle - start_angle
        num_values = int(angle_range / angle_resolution)
        
        if int(angle_range)% int(angle_resolution)==0:
            num_values=num_values+1

        self.laser_msgs_.ranges= [0.0]*num_values

        self.laser_msgs_.time_increment= float((angle_resolution / 10000.0) / 360.0 / (scan_frequency / 100.0))
        self.laser_msgs_.angle_increment= float((angle_resolution / 10000.0 * self.DEG2RAD))
        self.laser_msgs_.angle_min = float(start_angle / 10000.0 * self.DEG2RAD - 3.14 / 2)
        self.laser_msgs_.angle_max = float(stop_angle / 10000.0 * self.DEG2RAD - 3.14 / 2)
        self.laser_msgs_.scan_time = float(100.0 / scan_frequency)
        self.laser_msgs_.range_min=0.0
        self.laser_msgs_.range_max = 10.0

        self.counter_ =0.0
        self.amplitude_=1
        self.distance_ =0.0
        self.loop_=0

    
    
    def timer_callback(self):
        self.counter_=self.counter_+1
        self.loop_ =self.loop_+1
        self.distance_ = float(abs(self.amplitude_))

        for i in range(len(self.laser_msgs_.ranges)):
            self.laser_msgs_.ranges[i]=self.distance_

        dot = 1000/30
        step = self.loop_%dot
        s = step*dot
        if (step + 1) * dot < len(self.laser_msgs_.ranges):
            end = (step+1)*dot
        else:
            end = len(self.laser_msgs_.ranges)

        for i in range(int(s),int(end)):
            self.laser_msgs_.ranges[i]=0

        self.laser_msgs_._header.stamp= self.get_clock().now().to_msg()
        self.publisher_.publish(self.laser_msgs_)

            
        
        



def main(args=None):
    print('Hi from laser_scan_publisher_py.')
    rclpy.init(args=args)
    laserScanPublisherSim =LaserScanPublisherSim()
    rclpy.spin(laserScanPublisherSim)

    laserScanPublisherSim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
