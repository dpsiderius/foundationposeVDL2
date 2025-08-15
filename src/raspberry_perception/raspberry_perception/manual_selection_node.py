import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge

class ManualSelectionNode(Node):
    def __init__(self):
        super().__init__('manual_selection_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/color/image_raw', self.cb, 1)
        self.pub = self.create_publisher(RegionOfInterest, '/selected_roi', 10)
        self._img = None; self._drag=False; self._p0=None; self._p1=None
        cv2.namedWindow('select'); cv2.setMouseCallback('select', self.mouse)

    def cb(self, msg):
        self._last_header = msg.header
        self._img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        disp = self._img.copy()
        if self._p0 and self._p1:
            cv2.rectangle(disp, self._p0, self._p1, (0,255,0), 2)
        cv2.imshow('select', disp); cv2.waitKey(1)

    def mouse(self, event, x, y, *_):
        if event==cv2.EVENT_LBUTTONDOWN:
            self._drag=True; self._p0=(x,y); self._p1=(x,y)
        elif event==cv2.EVENT_MOUSEMOVE and self._drag:
            self._p1=(x,y)
        elif event==cv2.EVENT_LBUTTONUP and self._drag:
            self._drag=False; self._p1=(x,y)
            x0,y0 = self._p0; x1,y1 = self._p1
            x_off, y_off = min(x0,x1), min(y0,y1)
            w, h = abs(x1-x0), abs(y1-y0)
            if w>5 and h>5:
                roi = RegionOfInterest(x_offset=int(x_off), y_offset=int(y_off),
                                       width=int(w), height=int(h), do_rectify=False)
                self.pub.publish(roi)
                self.get_logger().info(f'Published ROI: {roi}')

def main():
    rclpy.init(); n=ManualSelectionNode()
    try: rclpy.spin(n)
    finally:
        cv2.destroyAllWindows(); n.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
