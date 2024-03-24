# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import re
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class BrojListener(Node):

    def __init__(self):
        super().__init__('broj_listener')
        self.subscription = self.create_subscription(
            String,
            'broj',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(String, 'kvadrat_broja', 10)
        time_period = 1
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.y = []
    

    def listener_callback(self, msg):
        self.y = list(map(int, re.findall('\d+', msg.data)))
        self.get_logger().info('I heard: "%s"' % msg.data)
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello world %d' % self.y[0]**2
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    broj_listener = BrojListener()

    rclpy.spin(broj_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    broj_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
