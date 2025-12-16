#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError

class DetecteurObjet:
    def __init__(self):
        rospy.init_node('red_object_detector_node', anonymous=True)
        self.bridge = CvBridge()

        self.largeur_image = 640
        self.seuil_aire_min = 50
        self.seuil_erreur_ignore = 2
        self.target_locked = False

        self.pub_erreur = rospy.Publisher('/object_error', Float32, queue_size=1)
        self.pub_taille = rospy.Publisher('/object_size', Float32, queue_size=1)

        rospy.Subscriber("/camera/rgb/image_raw", Image, self.rappel_image)

    def rappel_image(self, data):
        try:
            image_cv = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

        bas_rouge_1 = np.array([0, 100, 100])
        haut_rouge_1 = np.array([10, 255, 255])
        bas_rouge_2 = np.array([170, 100, 100])
        haut_rouge_2 = np.array([180, 255, 255])

        masque = cv2.inRange(hsv, bas_rouge_1, haut_rouge_1) + cv2.inRange(hsv, bas_rouge_2, haut_rouge_2)
        contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        erreur = 0.0
        taille = 0.0

        if contours:
            valid_contours = [c for c in contours if cv2.contourArea(c) > self.seuil_aire_min]

            if valid_contours:
                plus_petit_contour = min(valid_contours, key=cv2.contourArea)
                aire = cv2.contourArea(plus_petit_contour)

                M = cv2.moments(plus_petit_contour)
                if M["m00"] > 0:
                    cx = int(M['m10'] / M['m00'])
                    erreur_brute = cx - (self.largeur_image / 2)
                    
                    if abs(erreur_brute) > self.seuil_erreur_ignore:
                        erreur = erreur_brute

                    taille = aire
                    self.target_locked = True

        self.pub_erreur.publish(Float32(erreur))
        self.pub_taille.publish(Float32(taille))

        masque_petit = cv2.resize(masque, (320, 240))
        cv2.imshow("Masque Rouge", masque_petit)
        cv2.waitKey(3)

def main():
    node = DetecteurObjet()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
