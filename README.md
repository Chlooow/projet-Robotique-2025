# Vigilis : Robot de Vigilance (Projet ROS/OpenCV)

Ce projet a été réalisé dans le cadre de l’option **Robotique et aide au diagnostic** du Master 1 Informatique & Big Data (2025/2026) à l'Université Paris 8.

**Collaborateurs :** Tugba Bulut et Chloé Makoundou.

## Présentation du projet
L'objectif de **Vigilis** est de concevoir un système de surveillance autonome capable d'identifier une cible (un cylindre rouge) via une caméra, de la suivre en temps réel et de maintenir une distance de sécurité constante.

## Environnement Technique
* **Système d'exploitation :** Ubuntu 20.04.6
* **ROS :** Noetic.
* **Simulateur :** Gazebo avec un monde personnalisé (`projet.world`).
* **Robot :** TurtleBot3 Waffle.
* **Langage :** Python 3.
* **Traitement d'image :** OpenCV ( conversion HSV, masquage, calcul de centroïde).

## Architecture du Workspace (`catkin_ws_3`)
```text
catkin_ws_3/
└── src/
    └── robot/
        ├── launch/
        │   └── projet.launch       # Lance la simulation et les nœuds
        ├── scripts/
        │   ├── object_detector.py  # Nœud de détection (Vision)
        │   └── object_follower.py  # Nœud de commande (Mouvement)
        └── worlds/
            └── projet.world        # Environnement Gazebo
```

## Fonctionnement du Système

### 1. Perception (`red_object_detector_node`)
Le noeud analyse le flux `/camera/rgb/image_raw`:
* **Prétraitement :** Conversion en espace colorimétrique **HSV** pour limiter l'impact de la luminosité.
* **Segmentation :** Application de deux masques pour isoler le rouge (0-10 et 170-180 sur la teinte).
* **Calculs :** Extraction du centre de masse (centroïde) pour calculer l'erreur de centrage horizontale.

### 2. Commande (`object_follower_node`)
Le noeud pilote le robot sur `/cmd_vel` via un asservissement visuel:

* **Contrôle Proportionnel :** Gain  pour la vitesse angulaire.
* **Lissage :** Filtre passe-bas avec  pour éviter les mouvements saccadés.
* **États de comportement :**
* **Recherche :** Rotation à  si la cible est perdue.
* **Suivi actif :** Avance à  si la cible est détectée.
* **Arrêt de sécurité :** Stop complet si la surface de l'objet dépasse  pixels (proximité immédiate).

## Topics Utilisés

| Topic | Type | Rôle |
| --- | --- | --- |
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | Flux vidéo brut. |
| `/object_error` | `std_msgs/Float32` | Erreur horizontale (décalage). |
| `/object_size` | `std_msgs/Float32` | Taille de la cible (distance relative). |
| `/cmd_vel` | `geometry_msgs/Twist` | Commandes de mouvement. |

## Lancement

Pour démarrer la simulation et le suivi :

```bash
source devel/setup.bash
roslaunch robot projet.launch

```

