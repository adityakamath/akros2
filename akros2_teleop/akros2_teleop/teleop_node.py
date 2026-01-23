import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from akros2_teleop.joy_mode_handler import JoystickModeHandler
from akros2_teleop.twist_mixer_node import TwistMixer

def main(args=None):
    rclpy.init(args=args)

    try:
        joy_mode_handler = JoystickModeHandler(node_name='joy_mode_handler')
        twist_mixer = TwistMixer(node_name='twist_mixer')

        executor = MultiThreadedExecutor()
        executor.add_node(twist_mixer)
        executor.add_node(joy_mode_handler)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            twist_mixer.destroy_node()
            joy_mode_handler.destroy_node()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
