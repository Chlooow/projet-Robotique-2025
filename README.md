# Vigilis : Robot de Vigilance (Projet ROS/OpenCV)

Ce projet a été réalisé dans le cadre de l’option **Robotique et aide au diagnostic** du Master 1 Informatique & Big Data (2025/2026) à l'Université Paris 8.

**Collaborateurs :** Tugba Bulut et Chloé Makoundou.

## 🎯 Présentation du projet
L'objectif de **Vigilis** est de concevoir un système de surveillance autonome capable d'identifier une cible (un cylindre rouge) via une caméra, de la suivre en temps réel et de maintenir une distance de sécurité constante.

## 🛠 Environnement Technique
* **Système d'exploitation :** Ubuntu 20.04.6 (Noetic).
* **Simulateur :** Gazebo avec un monde personnalisé (`projet.world`).
* **Robot :** TurtleBot3 Waffle.
* **Langage :** Python 3.
* **Traitement d'image :** OpenCV ( conversion HSV, masquage, calcul de centroïde).

## 📂 Architecture du Workspace (`catkin_ws_3`)
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

