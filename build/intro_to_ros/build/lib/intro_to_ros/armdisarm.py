import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool

class ArmDisarm (Node):


    def __init__ (self):

        super().__init__("bluerov2_armdisarm")
        self.arm_service = self.create_client(CommandBool, "/mavros/cmd/arming")
    

    def arm_disarm_rov(self, arm):

        request = CommandBool.Request()
        request.value = arm
        future = self.arm_service.call_async(request)
        future.add_done_callback(self.arm_callback)
    

    def arm_callback(self, future):
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Successfully sent arm command")
            else:
                self.get_logger().info("Successfully sent arm command")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def main():

        rclpy.init()
        arm_disarm_node = ArmDisarm()
        arm_disarm_node.arm_disarm_rov(True)

        try:
            rclpy.spin(arm_disarm_node)
        except:
            pass
        finally:
            arm_disarm_node.arm_disarm_rov(False)
        arm_disarm_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
    

    if __name__ == "__main__":
        main()
