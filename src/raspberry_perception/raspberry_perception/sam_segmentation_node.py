import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge

class SamSegmentationNode(Node):
    def __init__(self):
        super().__init__('sam_segmentation_node')
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.cb_img, 1)
        self.sub_roi = self.create_subscription(RegionOfInterest, '/selected_roi', self.cb_roi, 10)
        self.pub_mask = self.create_publisher(Image, '/segmentation/mask', 10)
        self.last_img = None; self.last_msg = None; self.last_roi = None

    def cb_img(self, msg):
        self.last_msg = msg
        self.last_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.last_roi is not None:
            self.publish_roi_mask()

    def cb_roi(self, roi):
        self.last_roi = roi
        if self.last_img is not None:
            self.publish_roi_mask()

    def publish_roi_mask(self):
        img = self.last_img; roi = self.last_roi
        H, W = img.shape[:2]
        x, y, w, h = roi.x_offset, roi.y_offset, roi.width, roi.height
        x2, y2 = min(x+w, W), min(y+h, H)
        mask = np.zeros((H, W), np.uint8)
        mask[y:y2, x:x2] = 255  # placeholder mask = ROI box
        msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        msg.header = self.last_msg.header   # exact stamp + frame for FP sync
        self.pub_mask.publish(msg)
        self.get_logger().info('Published /segmentation/mask')

def main():
    rclpy.init(); n=SamSegmentationNode(); rclpy.spin(n); rclpy.shutdown()

if __name__=='__main__': main()
