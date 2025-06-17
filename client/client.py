import rospy
import cv2
import socket
import struct
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class RobotClient:
    def __init__(self):
        rospy.init_node('robot_ball_client')
        
        self.bridge = CvBridge()
        
        self.server_ip = '192.168.50.211' 
        self.server_port = 1489
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.image_sub = rospy.Subscriber("/front_camera/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        self.current_frame = None
        self.frame_lock = threading.Lock()
        
        self.connect_to_server()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.frame_lock:
                self.current_frame = cv_image
        except Exception as e:
            rospy.logerr(f"Image conversion error: {e}")

    def connect_to_server(self):
        max_retries = 5
        for attempt in range(max_retries):
            try:
                self.socket.connect((self.server_ip, self.server_port))
                rospy.loginfo(f"Connected to server at {self.server_ip}:{self.server_port}")
                return True
            except socket.error as e:
                if attempt == max_retries - 1:
                    rospy.logerr(f"Connection failed after {max_retries} attempts: {e}")
                    return False
                rospy.logwarn(f"Connection attempt {attempt + 1} failed, retrying...")
                rospy.sleep(2)

    def send_video_frames(self):
        rate = rospy.Rate(20)  
        
        while not rospy.is_shutdown():
            with self.frame_lock:
                if self.current_frame is None:
                    rate.sleep()
                    continue
                
                frame = self.current_frame.copy()
            
            try:
                _, img_encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                
                self.socket.sendall(struct.pack('>L', len(img_encoded)))
                self.socket.sendall(img_encoded.tobytes())
                
                self.socket.settimeout(0.1)
                try:
                    command = self.socket.recv(1024).decode()
                    if command:
                        self.process_command(command)
                except socket.timeout:
                    pass
                    
            except Exception as e:
                rospy.logerr(f"Communication error: {e}")
                break
            
            rate.sleep()

    def process_command(self, command):
        twist = Twist()
        if command == 'LEFT':
            twist.angular.z = 1
        elif command == 'RIGHT':
            twist.angular.z = -1
        
        self.cmd_vel_pub.publish(twist)

    def run(self):
        try:
            self.send_video_frames()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.socket.close()
            rospy.loginfo("Robot client shutdown")

if __name__ == '__main__':
    import threading
    client = RobotClient()
    client.run()
