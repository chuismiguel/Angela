import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Float32
from std_msgs.msg import ColorRGBA
import board
import neopixel

MAX_DISTANCE = 1.0
TH = 0.05

US_PIN = 26

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        self._pin = US_PIN
        self.max_distance = MAX_DISTANCE
        self.threshold_distance = TH
        self._distance = MAX_DISTANCE
        self.when_in_range = self.when_out_of_range = None

        # Create a publisher for the sensor distance
        self.publisher_ = self.create_publisher(Float32, 'angela/sensor_distance', 10)
        self.timer = self.create_timer(1.0, self.publish_distance)

        # Create a subscriber for the LED colors
        self.subscription = self.create_subscription(
            ColorRGBA,
            'angela/sensor_leds',
            self.led_callback,
            10
        )

        # Initialize the NeoPixel LEDs
        self.pixels = neopixel.NeoPixel(board.D12, 6)

    def get_distance(self):
            GPIO.setup(self._pin, GPIO.OUT)
            # set Trigger to HIGH
            GPIO.output(self._pin, False)
            time.sleep(0.000002)
            GPIO.output(self._pin, True)

            # set Trigger after 0.01ms to LOW
            time.sleep(0.00001)
            GPIO.output(self._pin, False)

            GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            StartTime = StopTime = TriggerTime = time.time()

            # save StartTime
            while GPIO.input(self._pin) == 0:
                StartTime = time.time()
                if StartTime - TriggerTime > 0.01:
                    return

            # save time of arrival
            while GPIO.input(self._pin) == 1:
                pass
            StopTime = time.time()

            # time difference between start and arrival
            TimeElapsed = StopTime - StartTime
            # multiply with the sonic speed (34300 cm/s)
            # and divide by 2, because there and back
            self._last_distance = self._distance
            self._distance = min(TimeElapsed * 171.5, self.max_distance)

            if self._distance > self.threshold_distance > self._last_distance:
                self.when_out_of_range and self.when_out_of_range()
            elif self._distance < self.threshold_distance < self._last_distance:
                self.when_in_range and self.when_in_range()

            return self._distance        

    def publish_distance(self):
        distance = self.get_distance()
        msg = Float32()
        msg.data = distance
        self.publisher_.publish(msg)

    def led_callback(self, msg):
        rgb_tuple = (int(msg.r), int(msg.g), int(msg.b))
        self.pixels.fill(rgb_tuple)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
