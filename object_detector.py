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
        rospy.init_node('object_detector', anonymous=True)
        self.pont = CvBridge()
        
        # --- PARAMÈTRES ADJUSTABLES ---
        self.largeur_image = 640  # Largeur de l'image (pour le calcul de l'erreur)
        self.seuil_aire_min = 500 # Seuil minimum de pixels pour filtrer le bruit
        self.seuil_erreur_ignore = 15 # NE PAS TOURNER si l'erreur est inférieure à 15 pixels

        # Publications (Publier l'erreur d'angle et la taille/zone)
        self.pub_erreur = rospy.Publisher('/object_error', Float32, queue_size=1)
        self.pub_taille = rospy.Publisher('/object_size', Float32, queue_size=1)
        
        # Abonnements
        self.abonnement_image = rospy.Subscriber("/camera/rgb/image_raw", Image, self.rappel_image)

    def rappel_image(self, donnees):
        try:
            image_cv = self.pont.imgmsg_to_cv2(donnees, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)
        
        # 🔴 Filtre de Couleur pour le ROUGE (Optimisé 170-180)
        bas_rouge_1 = np.array([0, 100, 100])
        haut_rouge_1 = np.array([10, 255, 255])
        bas_rouge_2 = np.array([170, 100, 100])
        haut_rouge_2 = np.array([180, 255, 255])

        masque_1 = cv2.inRange(hsv, bas_rouge_1, haut_rouge_1)
        masque_2 = cv2.inRange(hsv, bas_rouge_2, haut_rouge_2)
        masque = masque_1 + masque_2
        
        # Trouver les contours et le plus grand
        contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        erreur = 0.0
        taille = 0.0
        
        # --- LOGIQUE D'ALGORITHME ---
        if contours:
            plus_grand_contour = max(contours, key=cv2.contourArea)
            aire = cv2.contourArea(plus_grand_contour)
            
            if aire > self.seuil_aire_min: 
                M = cv2.moments(plus_grand_contour)
                if M["m00"] > 0:
                    cx = int(M['m10'] / M['m00'])
                    
                    # 1. Calcul de l'Erreur (Décentrement)
                    erreur_brute = cx - (self.largeur_image / 2)
                    taille = aire 
                    
                    # 2. Application de la Tolérance (Ignorer les petites erreurs)
                    if abs(erreur_brute) > self.seuil_erreur_ignore:
                        erreur = erreur_brute
                    else:
                        erreur = 0.0 # L'objet est suffisamment centré, pas besoin de tourner.
                    
                    # Dessin du contour et du centre (Optionnel)
                    cv2.drawContours(image_cv, [plus_grand_contour], -1, (0, 255, 0), 2)
                    cv2.circle(image_cv, (cx, int(image_cv.shape[0] / 2)), 5, (0, 0, 255), -1)

        # Publication des résultats
        self.pub_erreur.publish(Float32(erreur))
        self.pub_taille.publish(Float32(taille))
        
        rospy.loginfo("DETECTOR: Erreur: %.2f, Taille: %.2f (Tolérance: %d)", erreur, taille, self.seuil_erreur_ignore)

        cv2.imshow("DETECTOR: Masque Rouge", masque)
        cv2.waitKey(3)

def principal_detecteur():
    detecteur = DetecteurObjet()
    rospy.spin()

if __name__ == '__main__':
    try:
        principal_detecteur()
    except rospy.ROSInterruptException:
        pass
