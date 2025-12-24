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
    └── robot/                  # Package principal [cite: 66]
        ├── launch/
        │   └── projet.launch   # Lance Gazebo, le robot et les nœuds [cite: 297]
        ├── scripts/
        │   ├── object_detector.py  # Nœud de vision (Perception) [cite: 67]
        │   └── object_follower.py  # Nœud de contrôle (Décision/Action) [cite: 67]
        └── worlds/
            └── projet.world    # Environnement de simulation [cite: 78]
