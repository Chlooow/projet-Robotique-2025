# Projet Option Robotique - 2025/2026

Ce projet a été réalisé dans le cadre de l’option **Robotique** du Master 1 Informatique & Big Data.  

L’objectif est de développer un **robot TurtleBot3 Waffle** capable de **suivre un objet coloré** (ex : cube rouge) grâce à sa caméra et à un traitement d’image en temps réel via **ROS** et **OpenCV**.

## Scénario 1: Robot reconnaissant la couleur

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

#### 1. `object_detector.py`
Détecte l’objet coloré et calcule son erreur de position.

##### Fonctionnalités :
- Récupération du flux caméra  
- OpenCV : filtre de couleur (HSV)  
- Extraction du plus grand contour  
- Calcul du centre de l’objet  
- Publication de l’erreur de suivi

##### Topics :
| Rôle       | Topic                        | Type                |
|------------|------------------------------|---------------------|
| Subscribe  | `/camera/rgb/image_raw`      | sensor_msgs/Image   |
| Publish    | `/object_error`              | std_msgs/Float32    |
| Publish    | `/object_size`               | std_msgs/Float32    |

#### 2. `object_follower.py`
Contrôle le robot selon l’erreur détectée.

##### Fonctionnalités :
- Récupère l’erreur de position  
- Tourne à gauche/droite pour aligner l’objet  
- Avance si l’objet est loin  
- S’arrête si l’objet est trop proche


##### Topics :
| Rôle       | Topic             | Type                    |
|------------|-------------------|-------------------------|
| Subscribe  | `/object_error`   | std_msgs/Float32        |
| Subscribe  | `/object_size`    | std_msgs/Float32        |
| Publish    | `/cmd_vel`        | geometry_msgs/Twist     |

## Topics utilisés

| Topic                    | Description                              |
|--------------------------|------------------------------------------|
| `/camera/rgb/image_raw` | Flux vidéo de la caméra du robot         |
| `/object_error`         | Erreur horizontale (position de l’objet) |
| `/object_size`          | Taille de l’objet (distance approximée)  |
| `/cmd_vel`              | Commandes moteurs                        |

# Structure du package

## Collaborateurs :

Chloé Makoundou et Tugba Bulut



