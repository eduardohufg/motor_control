# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool
from numpy import sin, cos
from scipy.signal import square, sawtooth

#Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        # Declare the signal parameters and other variables
        self.declare_parameter('amplitude', 1.75)
        self.declare_parameter('omega', 1.0)
        self.declare_parameter('timer_period', 0.01)
        self.declare_parameter('type', 1)
        self.system_running = False

        # Retrieve the signal parameters
        self.amplitude = self.get_parameter('amplitude').value
        self.omega = self.get_parameter('omega').value
        self.timer_period = self.get_parameter('timer_period').value
        self.signal_type = self.get_parameter('type').value

        # Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float64, 'set_point', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        # Parameters callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Create a messages and variables to be used
        self.signal_msg = Float64()
        self.start_time = self.get_clock().now()

        #Create a service client for /EnableProcess
        self.cli = self.create_client(SetProcessBool, 'EnableProcess')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.get_logger().info("SetPoint Node Started \U0001F680")

    # Timer Callback: Generate and Publish Sine Wave Signal
    def timer_cb(self):
        # Calculate the elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9
        #  Generate the signal
        # Sine Wave
        if self.signal_type == 1:
            self.signal_msg.data = self.amplitude * sin(self.omega * elapsed_time)
        # Cosine Wave
        elif self.signal_type == 2:
            self.signal_msg.data = self.amplitude * cos(self.omega * elapsed_time)
        # Square Wave
        elif self.signal_type == 3:
            self.signal_msg.data = self.amplitude * square(self.omega * elapsed_time)
        # Triangular Wave
        elif self.signal_type == 4:
            self.signal_msg.data = self.amplitude * sawtooth(self.omega * elapsed_time, 0.5)
        # Falling Ramp Wave
        elif self.signal_type == 5:
            self.signal_msg.data = self.amplitude * sawtooth(self.omega * elapsed_time, 0.)
        # Rising Ramp Wave
        elif self.signal_type == 6:
            self.signal_msg.data = self.amplitude * sawtooth(self.omega * elapsed_time, 1.)
        # Unitary Step Signal
        elif self.signal_type == 7:
            elapsed_time = 0.0
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9
            if elapsed_time < 20.0:
                self.signal_msg.data = 0.0
            else:
                self.signal_msg.data = 1.0

        # Publish the signal
        self.signal_publisher.publish(self.signal_msg)
        

    # Send Request to Start/Stop Simulation
    def send_request(self, enable: bool):
        request = SetProcessBool.Request()
        request.enable = enable

        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)

    # Process the Service Response
    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.system_running = True
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.simulation_running = False
                self.get_logger().warn(f'Failure: {response.message}')
        except Exception as e:
            self.simulation_running = False
            self.get_logger().error(f'Service call failed: {e}')

    # Parameters Callback
    def parameters_callback(self, params):
        for param in params:
            # Amplitude parameter check
            if param.name == "amplitude":
                if (param.value < 0.0):
                    self.get_logger().warn("Amplitude cannot be negative. Invalid parameter value.")
                    return SetParametersResult(successful=False, reason="Amplitude cannot be negative")
                else:
                    self.amplitude = param.value
                    self.get_logger().info(f"Amplitude set to: {self.amplitude}")
            # Omega parameter check
            elif param.name == "omega":
                if (param.value < 0.0):
                    self.get_logger().warn("Omega cannot be negative. Invalid parameter value.")
                    return SetParametersResult(successful=False, reason="Omega cannot be negative")
                else:
                    self.omega = param.value
                    self.get_logger().info(f"Omega set to: {self.omega}")
            # Signal Type parameter check
            elif param.name == "type":
                if (param.value < 1 or param.value > 7):
                    self.get_logger().warn("Invalid Signal Type. Valid values are 1 to 7.")
                    return SetParametersResult(successful=False, reason="Invalid Signal Type")
                else:
                    self.signal_type = param.value
                    self.get_logger().info(f"Signal Type set to: {self.signal_type}")

        return SetParametersResult(successful=True)

#Main
def main(args=None):
    rclpy.init(args=args)
    set_point = SetPointPublisher()
    try: rclpy.spin(set_point)
    except KeyboardInterrupt: print("\nSetPoint Node terminated by the user!")
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
