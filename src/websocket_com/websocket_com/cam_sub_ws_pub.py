# Last edited: Feb. 17, 2023 by Annalisa Jarecki

import base64
import logging
import time
import roslibpy

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time

bridge = CvBridge()

# Configure logging
fmt = '%(asctime)s %(levelname)8s: %(message)s'
logging.basicConfig(format=fmt, level=logging.INFO)
log = logging.getLogger(__name__)

client = roslibpy.Ros(host='10.0.0.142', port=9090)
client.run()

publisher = roslibpy.Topic(client, '/ra_camera', 'sensor_msgs/CompressedImage')



class CamSubWsPub(Node):

    def __init__(self):
        super().__init__('cam_sub_ws_pub')
        
        self.publisher_ = self.create_publisher(CompressedImage, 'comp_im_topic', 10)
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        tic = time.time()
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg)
        #cmp_img = bridge.cv2_to_compressed_imgmsg(cv2_img)
        is_success, im_buf_arr = cv2.imencode(".jpg", cv2_img)
        byte_im = im_buf_arr.tobytes()
        encoded = base64.b64encode(byte_im).decode('ascii')
        publisher.publish(dict(format='jpeg', data=encoded))
        toc = time.time()
        print(toc-tic)
        time.sleep(0.2)
        
        
def main(args=None):
    rclpy.init(args=args)
    
    camsubwspub = CamSubWsPub()
    rclpy.spin(camsubwspub)
    
    #publisher.unadvertise()
    #client.terminate()



if __name__ == '__main__':
    main()

