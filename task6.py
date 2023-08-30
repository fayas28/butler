#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class OrderReceiver(Node):
    def __init__(self):
        super().__init__('order_receiver')
        self.subscription = self.create_subscription(
            String,
            '/order',
            self.order_callback,
            10
        )
        self.confirmation_subscription = self.create_subscription(
            String,
            '/confirmation',
            self.confirmation_callback,
            10
        )
        self.table_orders = []
        self.get_logger().info("Order Receiver node initialized")

    def order_callback(self, msg):
        order_info = msg.data
        self.get_logger().info(f"Received order: {order_info}")
        self.table_orders.append(order_info.split(',')[0])  # Extract table number
        
        if len(self.table_orders) == 3:
            self.process_orders()

    def process_orders(self):
        self.get_logger().info("Processing orders...")
        
        # Deliver orders to tables
        self.move_to_table()
        
        # Return home after all orders are delivered
        self.return_home()

    def deliver_to_kitchen(self):
        self.get_logger().info("Robot is on its way to the kitchen")
        time.sleep(3)  # Pause for 3 seconds to simulate reaching the kitchen
        self.get_logger().info("Robot has reached the kitchen")

    def move_to_table(self):
        for table_number in self.table_orders:
            self.get_logger().info(f"Robot is on its way to Table {table_number}")
            self.wait_for_confirmation(table_number)
            time.sleep(2)  # Pause for 2 seconds to simulate movement
            self.get_logger().info(f"Robot has reached Table {table_number}")

    def wait_for_confirmation(self, table_number):
        confirmation_received = False
        start_time = time.time()
        while not confirmation_received and (time.time() - start_time) < 5:
            rclpy.spin_once(self)
            time.sleep(0.1)  # Small delay to avoid excessive spinning

    def confirmation_callback(self, msg):
        self.get_logger().info(f"Received confirmation: {msg.data}")

    def return_home(self):
        self.get_logger().info("Robot is on its way back home")
        time.sleep(3)  # Pause for 3 seconds to simulate returning home
        self.get_logger().info("Robot has reached home position")
        time.sleep(3)  # Pause for 3 seconds before waiting for the next order
        self.table_orders = []  # Clear the table orders

    def run(self):
        self.get_logger().info("Waiting for next order")
        while rclpy.ok():
            rclpy.spin_once(self)  # Process any pending messages
            time.sleep(0.1)  # Small delay to avoid excessive spinning

def main(args=None):
    rclpy.init(args=args)
    order_receiver = OrderReceiver()

    order_receiver.run()  # Start waiting for and processing orders

    order_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
