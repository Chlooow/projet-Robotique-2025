#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np

class ControleurObjet:
    def __init__(self):
        rospy.init_node('object_follower_node', anonymous=True)

        rospy.Subscriber('/object_error', Float32, self.rappel_erreur)
        rospy.Subscriber('/object_size', Float32, self.rappel_taille)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vitesse_cmd = Twist()

        self.Kp_angle = 0.0003  # gain plus faible pour rotation douce
        self.taille_cible = 650000
        self.taille_min = 300

        self.erreur_angle = 0.0
        self.erreur_angle_lisse = 0.0  # moyenne glissante
        self.taille_objet = 0.0

        rospy.Timer(rospy.Duration(0.1), self.controler_robot)

    def rappel_erreur(self, data):
        self.erreur_angle = data.data

    def rappel_taille(self, data):
        self.taille_objet = data.data

    def controler_robot(self, event):
        alpha = 0.2  # lissage
        self.erreur_angle_lisse = alpha * self.erreur_angle + (1 - alpha) * self.erreur_angle_lisse

        # Limite de l'erreur angulaire
        erreur_max = 1000
        if abs(self.erreur_angle_lisse) > erreur_max:
            self.erreur_angle_lisse = np.sign(self.erreur_angle_lisse) * erreur_max

        if self.taille_objet <= self.taille_min:
            self.vitesse_cmd.linear.x = 0.0
            self.vitesse_cmd.angular.z = 0.2
            rospy.logwarn("FOLLOWER: Cible perdue, recherche...")

        elif self.taille_objet >= self.taille_cible:
            self.vitesse_cmd.linear.x = 0.0
            self.vitesse_cmd.angular.z = 0.0
            rospy.loginfo("FOLLOWER: Cible atteinte, arrêt complet.")

        else:
            self.vitesse_cmd.angular.z = -self.erreur_angle_lisse * self.Kp_angle
            self.vitesse_cmd.linear.x = 0.1
            rospy.loginfo("FOLLOWER: Suivi actif, angular: %.2f", self.vitesse_cmd.angular.z)

        self.pub_cmd_vel.publish(self.vitesse_cmd)

def main():
    node = ControleurObjet()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

