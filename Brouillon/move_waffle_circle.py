#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

pub = None
cmd = Twist()

def move_circle():
    rospy.loginfo("Le robot décrit un cercle.")

    # Vitesse linéaire = avance
    cmd.linear.x = 0.15

    # Vitesse angulaire = rotation légère pour faire un cercle
    cmd.angular.z = 0.4

    # Durée totale du mouvement
    duration = 10   # à ajuster pour la taille du cercle

    start_time = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(cmd)
        rospy.sleep(0.1)

    # Stop
    cmd.linear.x = 0
    cmd.angular.z = 0
    pub.publish(cmd)
    rospy.loginfo("Cercle terminé.")


if __name__ == "__main__":
    rospy.init_node("move_circle")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)
    move_circle()
