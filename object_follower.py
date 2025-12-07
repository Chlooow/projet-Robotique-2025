#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class ControleurObjet:

    def __init__(self):
        rospy.init_node('object_follower', anonymous=True)
        
        # Abonnements et Publications
        rospy.Subscriber('/object_error', Float32, self.rappel_erreur)
        rospy.Subscriber('/object_size', Float32, self.rappel_taille)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vitesse_cmd = Twist()
        
        # Paramètres de Contrôle (OPTIMISÉS)
        self.Kp_angle = 0.0001     # Gain proportionnel pour la rotation
        self.taille_cible = 700000  # Taille idéale (Seuil d'arrêt)
        self.taille_min = 2000     # Seuil minimum d'activation du suivi
        
        self.erreur_angle = 0.0
        self.taille_objet = 0.0
        
        # 10 Hz döngü ile kontrol et
        rospy.Timer(rospy.Duration(0.1), self.controler_robot) 

    def rappel_erreur(self, donnees):
        self.erreur_angle = donnees.data

    def rappel_taille(self, donnees):
        self.taille_objet = donnees.data

    def controler_robot(self, evenement):
        
        if self.taille_objet <= self.taille_min:
            # CAS 1: Obje kayboldu veya çok küçük (Arama dönüşü)
            rospy.logwarn("FOLLOWER: Cible perdue. Recherche...")
            self.vitesse_cmd.linear.x = 0.0 
            self.vitesse_cmd.angular.z = 0.5 # Yavaşça dönerek arama

        elif self.taille_objet > self.taille_cible:
            # CAS 2: Obje çok yakın (Dur)
            rospy.loginfo("FOLLOWER: Cible trop proche (%f). ARRET.", self.taille_objet)
            self.vitesse_cmd.linear.x = 0.0 
            self.vitesse_cmd.angular.z = 0.0

        else:
            # CAS 3: Obje bulundu ve takip mesafesinde (Takip)
            
            # Contrôle Angulaire (P-Kontrolü)
            self.vitesse_cmd.angular.z = -self.erreur_angle * self.Kp_angle
            
            # Contrôle Linéaire (Sabit hızla ilerle - Düzeltildi)
            self.vitesse_cmd.linear.x = 0.25 # Hızınızı 0.3'te sabitledik
            
            rospy.loginfo("FOLLOWER: Avance. Angle Z: %.2f", self.vitesse_cmd.angular.z)
        
        # Publier les commandes
        self.pub_cmd_vel.publish(self.vitesse_cmd)

def principal_controleur():
    controleur = ControleurObjet()
    rospy.spin()

if __name__ == '__main__':
    try:
        principal_controleur()
    except rospy.ROSInterruptException:
        pass
