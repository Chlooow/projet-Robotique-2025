import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Initialize the CvBridge
bridge = CvBridge()

def ImCamera(msg):
    """
    Callback function to convert the ROS Image message to an OpenCV image
    and display it.
    """
    try:
        # Convert the ROS Image message to a BGR OpenCV image
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    # Display the image in an OpenCV window
    cv2.imshow("Camera Feed", img)
    
    # Wait for 1 millisecond for a key press and allow the GUI to update
    cv2.waitKey(1)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node("camera_test_node", anonymous=True)
    
    # Subscribe to the image topic
    # Replace "/camera/rgb/image_raw" with the actual topic name your camera publishes to
    rospy.Subscriber("/camera/rgb/image_raw", Image, ImCamera)
    
    # Keep the node running until it's shut down
    rospy.spin()
    
    # Close all OpenCV windows when the ROS node shuts down
    cv2.destroyAllWindows()
