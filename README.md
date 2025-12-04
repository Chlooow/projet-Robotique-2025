# Projet Option Robotique - 2025/2026

Ce projet a été réalisé dans le cadre de l’option **Robotique** du Master 1 Informatique & Big Data.  

L’objectif est de développer un **robot TurtleBot3 Waffle** capable de **suivre un objet coloré** (ex : cube rouge) grâce à sa caméra et à un traitement d’image en temps réel via **ROS** et **OpenCV**.

Le robot doit :

1. **Détecter un objet spécifique** (principalement une couleur, ex. rouge).  
2. **Analyser l’image de la caméra** pour trouver la position de l’objet.  
3. **Calculer le décalage** entre l’objet et le centre de l’image.  
4. **Se déplacer automatiquement** pour suivre l’objet :
   - Avancer si l’objet est loin  
   - S’arrêter si l’objet est proche  
   - Tourner à gauche/droite pour garder l’objet centré  


# Objectif général

Le robot doit :

1. **Détecter un objet spécifique** (principalement une couleur, ex. rouge).  
2. **Analyser l’image de la caméra** pour trouver la position de l’objet.  
3. **Calculer le décalage** entre l’objet et le centre de l’image.  
4. **Se déplacer automatiquement** pour suivre l’objet :
   - Avancer si l’objet est loin  
   - S’arrêter si l’objet est proche  
   - Tourner à gauche/droite pour garder l’objet centré
  
# Pipeline de traitement

1. La caméra du TurtleBot3 publie un flux vidéo sur `/camera/rgb/image_raw`.  
2. Un noeud ROS utilisant OpenCV applique :
   - conversion vers HSV  
   - filtrage de la couleur  
   - extraction du contour de l’objet  
   - calcul du centre (cx, cy)  
3. L’erreur horizontale `cx - center_x` sert à corriger la rotation du robot.  
4. Le robot se déplace via commandes sur `/cmd_vel`.

# Architecture ROS

Le système repose sur **deux nœuds principaux** : 
## Collaborateurs :

Chloé Makoundou et Tugba Bulut

## Scénario 3 : Robot suiveur de ligne
- La caméra capture une image du sol.
- Un nœud ROS traite l’image via OpenCV.
- Le robot calcule un erreur de position de la ligne
- Le robot avance en corrigeant en permanence sa trajectoire

