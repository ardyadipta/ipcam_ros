#!/usr/bin/env python
import cv2
import urllib 
import numpy as np
from sensor_msgs.msg import Image 
import roslib
import sys
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError 
from sensor_msgs.msg import CameraInfo
import camera_info_manager
import argparse

class IPCam(object):
    def __init__(self, url, config):
        # Logging if the camera is opened or not
        try:
            self.stream=urllib.urlopen(url)
            rospy.loginfo('Opened camera stream: ' + str(url))
        except:
            rospy.logerr('Unable to open camera stream: ' + str(url))
            sys.exit() #'Unable to open camera stream')
            
        # Initialize camera variables    
        self.bytes=''
        self.width = 640
        self.height = 480
        self.frame_id = 'camera'
        self.image_pub = rospy.Publisher("/camera/image_raw", Image)
        self.cinfo = camera_info_manager.CameraInfoManager(cname = 'camera', url = config)
        self.cinfo.loadCameraInfo()         # required before getCameraInfo()
        self.caminfo_pub = rospy.Publisher("/camera/camera_info", CameraInfo)
        self.bridge = CvBridge()

    def publishCameraInfoMsg(self):
        '''Publish camera info manager message'''
        cimsg = self.cinfo.getCameraInfo()
        cimsg.header.stamp = rospy.Time.now()
        cimsg.header.frame_id = self.frame_id
        cimsg.width = self.width
        cimsg.height = self.height
        self.caminfo_pub.publish(cimsg)

def main():
    # Parse the arguments
    parser = argparse.ArgumentParser(prog='ipcam.py', description='reads a given url string and dumps it to a ros_image topic')
    parser.add_argument('-g', '--gui', action='store_true', help='show a GUI of the camera stream') 
    parser.add_argument('-c', '--config', default='PACKAGE://ipcam/config/calibration.yaml', help='camera calibration file URL') 
    parser.add_argument('url', help='camera stream url to parse')
    args, unknown = parser.parse_known_args()
    
    # initialize ROS node
    rospy.init_node('ip_camera', anonymous=True)
    
    # instantiate IPCam object
    ipcam = IPCam(args.url, args.config)
    
    # Executive loop
    while not rospy.is_shutdown():
        ipcam.bytes += ipcam.stream.read(1024)
        a = ipcam.bytes.find('\xff\xd8')
        b = ipcam.bytes.find('\xff\xd9')
        if a!=-1 and b!=-1:
            jpg = ipcam.bytes[a:b+2]
            ipcam.bytes= ipcam.bytes[b+2:]
            
            # Convert bytes to image
            i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
            
            # Publish image into ROS image message
            if i is not None:
                image_message = ipcam.bridge.cv2_to_imgmsg(i, "bgr8")
                image_message.header.stamp = rospy.Time.now()
                image_message.header.frame_id = ipcam.frame_id
                ipcam.image_pub.publish(image_message)
                ipcam.publishCameraInfoMsg()
                
                # For debugging
                #print "height: ", image_message.height
                #print "width: ", image_message.width
                #print "encoding: ", image_message.encoding
                #print "step: ", image_message.step
                #print "header: ", image_message.header
                
                # Show the images 
                if args.gui:
                    cv2.imshow('IP Camera Input',i)
                if cv2.waitKey(1) ==27: # wait until ESC key is pressed in the GUI window to stop it
                    exit(0) 
    
if __name__ == '__main__':
    main()
