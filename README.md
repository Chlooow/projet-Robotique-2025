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


Voici le code Markdown complet de votre fichier `README.md`, structuré et prêt à être copié-collé :

```markdown
# Vigilis : Robot de Vigilance (Projet ROS/OpenCV)

[cite_start]Ce projet a été réalisé dans le cadre de l’option **Robotique et aide au diagnostic** du Master 1 Informatique & Big Data (2025/2026) à l'Université Paris 8[cite: 1, 25].

[cite_start]**Collaborateurs :** Tugba Bulut et Nsonde Chloé Makoundou[cite: 1].

## 🎯 Objectif du Projet
[cite_start]Développer un système de surveillance autonome capable d'identifier une "menace" (représentée par un cylindre rouge) à l'aide d'une caméra, de la suivre en temps réel et de maintenir une distance de sécurité constante[cite: 30, 31, 34].

## 🛠 Environnement Technique
* [cite_start]**Système :** Ubuntu 20.04.6 (Noetic)[cite: 41, 73].
* [cite_start]**Simulation :** Gazebo avec un monde personnalisé (`projet.world`)[cite: 49, 78].
* [cite_start]**Robot :** TurtleBot3 Waffle (choisi pour sa caméra intégrée)[cite: 59, 60].
* [cite_start]**Langage :** Python 3[cite: 172].
* [cite_start]**Bibliothèques :** OpenCV (traitement d'image) et CvBridge (interface ROS)[cite: 71, 73].

## 📂 Structure du Workspace (`catkin_ws_3`)
```text
catkin_ws_3/
└── src/
    [cite_start]└── robot/                  # Package principal [cite: 66]
        ├── launch/
        [cite_start]│   └── projet.launch   # Lance Gazebo, le robot et les nœuds [cite: 297]
        ├── scripts/
        [cite_start]│   ├── object_detector.py  # Nœud de vision (Perception) [cite: 67]
        [cite_start]│   └── object_follower.py  # Nœud de contrôle (Décision/Action) [cite: 67]
        └── worlds/
            [cite_start]└── projet.world    # Environnement de simulation [cite: 78]

```

## Fonctionnement du Système

### 1. Perception (`red_object_detector_node`)

Le nœud analyse le flux `/camera/rgb/image_raw`:

* 
**Prétraitement :** Conversion en espace colorimétrique **HSV** pour limiter l'impact de la luminosité.


* 
**Segmentation :** Application de deux masques pour isoler le rouge (0-10 et 170-180 sur la teinte).


* 
**Calculs :** Extraction du centre de masse (centroïde) pour calculer l'erreur de centrage horizontale.



### 2. Commande (`object_follower_node`)

Le nœud pilote le robot sur `/cmd_vel` via un asservissement visuel:

* 
**Contrôle Proportionnel :** Gain  pour la vitesse angulaire.


* 
**Lissage :** Filtre passe-bas avec  pour éviter les mouvements saccadés.


* **États de comportement :**
* 
**Recherche :** Rotation à  si la cible est perdue.


* 
**Suivi actif :** Avance à  si la cible est détectée.


* 
**Arrêt de sécurité :** Stop complet si la surface de l'objet dépasse  pixels (proximité immédiate).





## Topics Utilisés

| Topic | Type | Rôle |
| --- | --- | --- |
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | Flux vidéo brut.

 |
| `/object_error` | `std_msgs/Float32` | Erreur horizontale (décalage).

 |
| `/object_size` | `std_msgs/Float32` | Taille de la cible (distance relative).

 |
| `/cmd_vel` | `geometry_msgs/Twist` | Commandes de mouvement.

 |

## Lancement

Pour démarrer la simulation et le suivi :

```bash
source devel/setup.bash
roslaunch robot projet.launch

```
