# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult

#Class Definition
class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        # Declare parameters for the PID controller
        # Proportional Gain
        self.declare_parameter('Kp', 0.09)
        # Integral Gain
        self.declare_parameter('Ki', 0.8)
        # Derivative Gain
        self.declare_parameter('Kd', 0.002)
        # Sample Time
        self.declare_parameter('Ts', 0.01)

        # Retrieve the PID controller parameters
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.Ts = self.get_parameter('Ts').value

        # Declare other variables
        self.error = 0.0
        self.previous_error = 0.0 
        self.integral_error = 0.0 
        self.derivative_error = 0.0 
        self.set_point = 0.0
        self.motor_output_y = 0.0
        self.control_input_u = 0.0

        # Set the message
        self.motor_input_u_msg = Float64()

        # Define publishers, subscribers and timers
        self.create_subscription(Float64, 'set_point', self.set_point_callback, 10)
        self.create_subscription(Float64, 'motor_output_y', self.motor_output_y_callback, 10)
        self.controller_pub = self.create_publisher(Float64, 'motor_input_u', 10)
        self.create_timer(self.Ts, self.controller_callback)

        # Parameters callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Node start
        self.get_logger().info("Motor Controller Node Started \U0001F680")

    # Set point subscriber callback
    def set_point_callback(self, msg):
        self.set_point = msg.data

    # Motor output subscriber callback
    def motor_output_y_callback(self, msg):
        self.motor_output_y = msg.data

    # Parameters callback
    def parameters_callback(self, params):
        for param in params:
            # Propotional gain parameter check
            if param.name == "Kp":
                # Check if its negative, if so, return an error
                if (param.value < 0.0):
                    self.get_logger().warn("Kp cannot be negative. Invalid parameter value.")
                    return SetParametersResult(successful=False, reason="Kp cannot be negative")
                else:
                    self.Kp = param.value
                    self.get_logger().info(f"Kp set to: {self.Kp}")

            # Integral gain parameter check
            elif param.name == "Ki":
                # Check if its negative, if so, return an error
                if (param.value < 0.0):
                    self.get_logger().warn("Ki cannot be negative. Invalid parameter value.")
                    return SetParametersResult(successful=False, reason="Ki cannot be negative")
                else:
                    self.Ki = param.value
                    self.get_logger().info(f"Ki set to: {self.Ki}")

            # Derivative gain parameter check
            elif param.name == "Kd":
                # Check if its negative, if so, return an error
                if (param.value < 0.0):
                    self.get_logger().warn("Kd cannot be negative. Invalid parameter value.") 
                    return SetParametersResult(successful=False, reason="Kd cannot be negative")
                else:
                    self.Kd = param.value
                    self.get_logger().info(f"Kd set to: {self.Kd}")

        return SetParametersResult(successful=True)
    
    # Controller callback
    def controller_callback(self):
        # Calculate the error
        self.error = self.set_point - self.motor_output_y

        # Integral error calculation (using trapezoidal rule)
        self.integral_error += ((self.error + self.previous_error) * self.Ts) / 2

        # Derivative error calculation
        self.derivative_error = (self.error - self.previous_error) / self.Ts

        # Calculate the control input using the PID formula
        self.control_input_u = self.Kp * self.error + self.Ki * self.integral_error + self.Kd * self.derivative_error

        # Publish the control input
        self.motor_input_u_msg.data = self.control_input_u
        self.controller_pub.publish(self.motor_input_u_msg)

        # Update the previous error for the next cycle
        self.previous_error = self.error
        
# Main
def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    try: rclpy.spin(motor_controller)
    except KeyboardInterrupt: print("\nController Node terminated by the user!")
    finally:
        motor_controller.destroy_node()
        rclpy.try_shutdown()

# Execute Node
if __name__ == '__main__':
    main()
