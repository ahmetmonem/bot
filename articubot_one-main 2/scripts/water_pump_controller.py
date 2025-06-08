#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import time

class WaterPumpController(Node):
    def __init__(self):
        super().__init__('water_pump_controller')
        
        # Subscriber for pump control commands
        self.pump_subscriber = self.create_subscription(
            Bool,
            'water_pump_control',
            self.pump_control_callback,
            10
        )
        
        # Service for manual pump control
        self.pump_service = self.create_service(
            SetBool,
            'pump_control_service',
            self.pump_service_callback
        )
        
        # Publisher for pump status
        self.pump_status_publisher = self.create_publisher(Bool, 'water_pump_status', 10)
        
        # Pump state
        self.pump_active = False
        self.last_activation_time = 0
        self.pump_timeout = 5.0  # Maximum continuous pump time (seconds)
        
        # Timer for status publishing
        self.status_timer = self.create_timer(0.5, self.publish_status)
        
        self.get_logger().info('Water Pump Controller Node Started')

    def pump_control_callback(self, msg):
        """Handle pump control commands from fire follower"""
        if msg.data and not self.pump_active:
            self.activate_pump()
        elif not msg.data and self.pump_active:
            self.deactivate_pump()

    def pump_service_callback(self, request, response):
        """Handle manual pump control service requests"""
        if request.data:
            success = self.activate_pump()
            response.success = success
            response.message = "Pump activated" if success else "Failed to activate pump"
        else:
            success = self.deactivate_pump()
            response.success = success
            response.message = "Pump deactivated" if success else "Failed to deactivate pump"
        
        return response

    def activate_pump(self):
        """Activate the water pump"""
        try:
            if not self.pump_active:
                self.pump_active = True
                self.last_activation_time = time.time()
                self.get_logger().info('Water pump ACTIVATED')
                
                # Here you would add actual hardware control code
                # For example: GPIO control, serial communication, etc.
                # Example:
                # self.control_pump_hardware(True)
                
                return True
        except Exception as e:
            self.get_logger().error(f'Failed to activate pump: {str(e)}')
            return False
        
        return False

    def deactivate_pump(self):
        """Deactivate the water pump"""
        try:
            if self.pump_active:
                self.pump_active = False
                self.get_logger().info('Water pump DEACTIVATED')
                
                # Here you would add actual hardware control code
                # For example: GPIO control, serial communication, etc.
                # Example:
                # self.control_pump_hardware(False)
                
                return True
        except Exception as e:
            self.get_logger().error(f'Failed to deactivate pump: {str(e)}')
            return False
        
        return False

    def control_pump_hardware(self, activate):
        """
        Control the actual pump hardware
        This is where you would implement the actual hardware interface
        """
        # Example implementations:
        
        # For GPIO control (Raspberry Pi):
        # import RPi.GPIO as GPIO
        # PUMP_PIN = 18
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(PUMP_PIN, GPIO.OUT)
        # GPIO.output(PUMP_PIN, GPIO.HIGH if activate else GPIO.LOW)
        
        # For Arduino serial communication:
        # import serial
        # arduino = serial.Serial('/dev/ttyUSB0', 9600)
        # command = "PUMP_ON\n" if activate else "PUMP_OFF\n"
        # arduino.write(command.encode())
        
        # For now, just log the action
        action = "ON" if activate else "OFF"
        self.get_logger().info(f'Hardware pump control: {action}')

    def publish_status(self):
        """Publish current pump status"""
        # Check for pump timeout (safety feature)
        if self.pump_active:
            current_time = time.time()
            if current_time - self.last_activation_time > self.pump_timeout:
                self.get_logger().warn('Pump timeout reached, deactivating for safety')
                self.deactivate_pump()
        
        # Publish status
        status_msg = Bool()
        status_msg.data = self.pump_active
        self.pump_status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    water_pump_controller = WaterPumpController()
    
    try:
        rclpy.spin(water_pump_controller)
    except KeyboardInterrupt:
        # Make sure pump is turned off when shutting down
        water_pump_controller.deactivate_pump()
    finally:
        water_pump_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()