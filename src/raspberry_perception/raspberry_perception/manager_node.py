import rclpy, asyncio
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import RegionOfInterest
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci  # replace later with TrackObject.action

class Manager(Node):
    def __init__(self):
        super().__init__('manager_node')
        self.state = 'IDLE'
        self.pose_sub = self.create_subscription(PoseStamped, '/object_pose', self.cb_pose, 10)
        self.roi_sub  = self.create_subscription(RegionOfInterest, '/selected_roi', self.cb_roi, 10)
        self.srv = self.create_service(Trigger, '/reset_selection', self.on_reset)
        self.action = ActionServer(self, Fibonacci, 'track_object', self.execute_cb)
        self.get_logger().info('Manager ready: IDLE')

    async def execute_cb(self, goal_handle):
        self.get_logger().info('Goal accepted → AWAITING_SELECTION')
        self.state = 'AWAITING_SELECTION'
        while rclpy.ok() and self.state != 'TRACKING_POSE':
            await asyncio.sleep(0.1)
        goal_handle.succeed()
        return Fibonacci.Result(sequence=[0])

    def cb_roi(self, _):
        if self.state in ['AWAITING_SELECTION', 'IDLE']:
            self.state = 'SEGMENTING'; self.get_logger().info('ROI → SEGMENTING')

    def cb_pose(self, _):
        if self.state in ['SEGMENTING', 'AWAITING_SELECTION']:
            self.state = 'TRACKING_POSE'; self.get_logger().info('Pose → TRACKING_POSE')

    def on_reset(self, req, res):
        self.state = 'AWAITING_SELECTION'
        res.success = True; res.message = 'Reset to AWAITING_SELECTION'
        return res

def main():
    rclpy.init(); n=Manager(); rclpy.spin(n); rclpy.shutdown()
if __name__=='__main__': main()
