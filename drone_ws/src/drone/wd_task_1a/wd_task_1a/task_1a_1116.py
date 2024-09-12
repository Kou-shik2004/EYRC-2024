import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
import time
from math import pi
class TurtleCommandNode(Node):

    def __init__(self):
        super().__init__('turtle_task1a')

        self.pub = self.create_publisher(Twist,'/turtle1/cmd_vel', 10)

        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.setpen_client = self.create_client(SetPen, '/turtle1/set_pen')

        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("teleport service not available. waiting...")
        while not self.setpen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("setpen service not available. waiting...")
        
        self.vel_msg = Twist()

    def teleport(self,x,y,theta):

        request = TeleportAbsolute.Request()

        request.x = x
        request.y = y
        request.theta = theta


        future = self.teleport_client.call_async(request)
        rclpy.spin_until_future_complete(self,future=future)

        if future.result() is not None:
            self.get_logger().info("teleported")
        else:
            self.get_logger().info("teleport call failed")

    def set_pen(self,r,g,b,width,off):

        request = SetPen.Request()

        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = self.setpen_client.call_async(request)
        rclpy.spin_until_future_complete(self,future=future)

        if future.result() is not None:
            self.get_logger().info("pen set")
        else:
            self.get_logger().info("set pen call failed")

    def drawcircle(self, r, s, cx=0.0, cy=0.0):

        self.set_pen(255,255,255,3,True)
        self.teleport(cx,cy-r,0.0)
        self.set_pen(255,255,255,3,False)

        self.vel_msg.linear.x = s
        self.vel_msg.angular.z = s/r

        i = 0
        while i<int(2*r*pi/(s*0.1))+2:
            self.pub.publish(self.vel_msg)
            time.sleep(0.1)
            i+=1
        
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        time.sleep(1.0)

    def drawline(self, a, b, c, d):
        self.set_pen(255,255,255,3,True)
        self.teleport(a,b,0.0)
        self.set_pen(255,255,255,3,False)
        self.teleport(c,d,0.0)
        
    
    def run(self):
        self.drawcircle(1.0, 3.0, 2.0, 2.0)
        self.drawcircle(1.0, 3.0, 2.0, 8.0)
        self.drawcircle(1.0, 3.0, 8.0, 8.0)
        self.drawcircle(1.0, 3.0, 8.0, 2.0)
        
        self.drawline(2.0,8.0,4.0,6.0)
        self.drawline(2.0,2.0,4.0,4.0)
        self.drawline(8.0,2.0,6.0,4.0)
        self.drawline(8.0,8.0,6.0,6.0)
        self.drawline(3.0,5.0,5.0,7.0)
        self.drawline(5.0,7.0,7.0,5.0)
        self.drawline(7.0,5.0,5.0,3.0)
        self.drawline(5.0,3.0,3.0,5.0)

        self.set_pen(0,0,0,3,True)
        self.teleport(5.0,5.0,0.0)

def main(args=None):
    rclpy.init(args=args)

    turtlecommand = TurtleCommandNode()

    try:
        turtlecommand.run()
    finally:
        turtlecommand.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()