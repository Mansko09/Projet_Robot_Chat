# Projet Robot Chat

**ESE – Projet de groupe**

## Introduction

Dans le cadre de la spécialité de 3e année ESE à l'ENSEA, ce projet a pour objectif la conception complète d’un **robot mobile autonome à deux roues motrices**, destiné à participer à un jeu de type **chat et souris** face aux robots des autres groupes. Ce projet a été réalisé par Florian, David, Danilo et Mantou.
Au-delà de l’aspect ludique, ce projet mobilise des compétences variées en **électronique**, **mécanique** et **logiciel embarqué temps réel**, avec des contraintes fortes de sécurité (anti-chute), de réactivité et de robustesse matérielle.

Le robot, nommé **Félix**, doit être capable de se déplacer de manière autonome, de détecter son environnement, de réagir à des événements physiques (vide, chocs) et d’adapter dynamiquement son comportement.

---

## Objectifs fonctionnels

Pour atteindre ce comportement, plusieurs fonctions clés doivent être mises en place :

1. **Déplacement**

   * Roues + moteurs
   * Drivers de moteurs (PWM via timers)
   * Encodeurs (lecture via timers)

2. **Anti-chute**

   * Détection des bords avec capteur **ToF** (I²C)

3. **Détection des adversaires**

   * Capteur **LIDAR** pour repérer d’autres robots

4. **Détection de collisions**

   * Échange de rôles lors d’un choc (capteur **accéléromètre**)

5. **Communication**

   * Liaison **Bluetooth** (STM32WB55CEU6)

6. **Alimentation**

   * Batterie et gestion d’énergie (7.2 V → régulateurs 5 V & 3.3 V)

---

## Composants principaux

* **Microcontrôleur** : STM32WB55CEU6 (ARM Cortex-M4, BLE intégré)
* **Quartz** : 32 MHz 
* **Connecteur SWD/STLink** : (Farnell : 3226055)
* **Moteurs** : DFRobot FIT0520 ou FIT0521
* **Alimentation** : Batterie NiMH 7.2 V 1.3 Ah (RS : 777-0377)

  * Régulateur 5 V : MP1475DJ-LF-P
  * Régulateur 3.3 V : BU33SD5WG-TR
* **Drivers moteurs** : ZXBM5210 (un par roue)
* **Capteurs** :

  * LIDAR YDLIDAR X2
  * Accéléromètre ADXL343BCCZ-RL
  * Capteur ToF anti-chute
  * Multiplexeur : TCA9548A
* **Connecteurs** : JST 2.54 mm
* **Interface utilisateur** :

  * LED rouge/verte
  * Boutons poussoirs Wurth 430182070816 (6×6 mm, H9.5 mm)
  * Interrupteur ON/OFF Wurth 472121020311

---

## Conception réalisée

1. **Électronique**

   * Schéma électrique et PCB personnalisé
   * Routage, fabrication et soudure des composants

2. **Mécanique**

   * Conception du châssis sur **Onshape**
   * Intégration du PCB, moteurs, capteurs et batterie

3. **Logiciel embarqué**

   * Contrôle moteurs et capteurs

## Avancement du projet

* [x] Choix des composants et architecture générale
* [x] Schéma et routage PCB
* [x] Impression 3D du robot
* [x] Développement logiciel de base
* [ ] Intégration LIDAR et logique “chat/souris”

## Architecture générale 

<img width="851" height="541" alt="image" src="https://github.com/user-attachments/assets/b1c18867-8e34-47c1-b6d0-efc3a15b8947" />

## Structure du Code

<img width="931" height="437" alt="image" src="https://github.com/user-attachments/assets/beefe881-a0c7-453a-90d6-29a493705993" />

### 1. Cerveau et Communication

Nous avons retenu le microcontrôleur **STM32WB55CEU6**, un choix motivé par des considérations de performance, de modularité et de fiabilité.

- **Mobilité et pilotage des drivers**  
  Chaque moteur est commandé par un driver piloté en PWM. Le logiciel intègre une gestion avancée des rampes de vitesse avec des profils asymétriques : accélération progressive et freinage brutal. Cette approche permet de préserver les moteurs tout en assurant un arrêt d’urgence immédiat lors de la détection de vide.  
  Les modes de freinage exploitent le **BRAKE_MODE** (court-circuit des moteurs) via le pont en H afin de garantir une immobilisation quasi instantanée sur les bords de table.

- **Intelligence embarquée et navigation**  
  Grâce à l’odométrie différentielle et aux encodeurs, la position $(x, y)$ ainsi que l’orientation $(\theta)$ du robot sont calculées en temps réel, permettant une navigation autonome précise.

- **Architecture logicielle sous FreeRTOS**  
  Le logiciel est structuré autour de plusieurs tâches concurrentes :
  - **Task_TOF** : mise à jour de la variable `vide` toutes les 20 ms, indiquant l’identifiant du capteur ToF détectant un vide ;
  - **Task_Motor** : gestion de l’odométrie et application des signaux PWM toutes les 10 ms ;
  - **Task_Control** : tâche décisionnelle principale, exécutée toutes les 20 ms, implémentant une machine à états finis (FSM) pilotant le comportement global du robot.

  **Rôle détaillé de la Task_Control**  
  La tâche **Task_Control** agit comme le véritable « cerveau » du robot. Elle fonctionne de manière asynchrone par rapport à la commande des moteurs et s’appuie sur une **Machine à États Finis (FSM)** pour orchestrer les actions de Félix en fonction des événements extérieurs.

  1. **Gestion de l’état global (Start / Stop)**  
     Avant toute logique de déplacement, la tâche surveille le flag `AccData`, mis à jour par l’accéléromètre via une interruption matérielle.  
     Un choc détecté agit comme un interrupteur logiciel (*toggle*) :
     - passage à l’état **Inactif** : LED rouge allumée, moteurs coupés ;
     - passage à l’état **Actif** : LED verte allumée, lancement de la mission.

  2. **Séquençage des déplacements (FSM)**  
     Lorsque le robot est actif, il suit un cycle comportemental conçu pour la survie et l’exploration :
     - **MOVE_FWD (marche avant)** : état par défaut. La tâche surveille en continu le registre `vide`. Dès qu’une bordure est détectée par un capteur ToF, une transition immédiate vers l’état de freinage est imposée.
     - **MOVE_BRAKE (sécurité)** : état intermédiaire déclenchant un arrêt d’urgence, voire une brève impulsion inverse, afin de compenser l’inertie et empêcher le dépassement du bord.
     - **MOVE_BACKWARD & MOVE_TURN** : une fois le robot immobilisé, une manœuvre de dégagement est engagée. Celle-ci est temporisée par `timer_state` et permet au robot de reculer, de modifier son orientation, puis de revenir automatiquement à l’état **MOVE_FWD**.

- **Perception sensorielle anti-chute**  
  Quatre capteurs **ToF VL53L0X** surveillent en permanence les angles du robot afin de détecter la présence de vide.

- **Détection de choc**  
  L’accéléromètre **ADXL343** est configuré en mode *Single Tap*. Un choc mécanique déclenche une interruption matérielle (EXTI) provoquant l’arrêt immédiat du robot et l’allumage de la LED rouge. Un second choc relance le système et active la LED verte.

## Ce qui fonctionne

- Déplacement fluide à une vitesse donnée du robot
- Détection de vide par les capteurs **ToF**
- Détection de choc par l’**accéléromètre**
- Gestion des LED d’état
- Changement de comportement en fonction :
  - de la détection de vide ;
  - de la détection de choc (Start / Stop)
 
## Ce qui ne fonctionne pas / partiellement

- **Bluetooth** : dysfonctionnement lié à un problème de quartz, empêchant la communication BLE
- **LIDAR** : matériel fonctionnel, mais logique logicielle incomplète
- **Asservissement** : implémenté mais non testé

## Challenges techniques rencontrés

### 1. Routage du PCB
Le premier défi a été le routage de la carte électronique :
- placement optimal des composants dans un espace réduit ;
- maintien des condensateurs de découplage au plus près du microcontrôleur ;
- isolation stricte de l’antenne Bluetooth ;
- optimisation des plans de masse ;
- éviter toute structure pouvant agir comme une antenne parasite.

### 2. Programmation du comportement du robot
Coder le comportement de Félix a été un défi majeur, non pas par la complexité de chaque fonction, mais par la nécessité de synchroniser une **logique** avec des **contraintes physiques réelles**.

#### a. Gestion du temps réel (priorités FreeRTOS)
Le robot ne doit jamais tomber, même lorsqu’il exécute d’autres calculs.
- **Problème** : la lecture des ToF via I²C est lente, alors que les moteurs nécessitent des mises à jour très fréquentes.
- **Solution** : architecture préemptive avec une priorité plus élevée pour la tâche ToF. En cas de détection de bord, elle interrompt la logique de contrôle pour forcer l’arrêt immédiat.

#### b. Passage du “tout ou rien” aux rampes (inertie)
- **Problème** : arrêts trop brutaux/lents provoquant des contraintes mécaniques (robot trop lourd à l'avant) ou un arrêt avec une roue en dehors de la table.
- **Solution** : implémentation de rampes asymétriques dans `Motor_UpdateSpeed` :
  - accélération douce (100 unités) ;
  - freinage très rapide (1000 unités).

Ce compromis entre protection mécanique et réactivité a nécessité de nombreux réglages.

#### c. Machine à États
- **Problème** : risque de blocage dans des situations critiques (coin de table, détections multiples).
- **Solution** : FSM rigoureuse avec des conditions de sortie strictes pour chaque état  
  (MOVE_FWD, MOVE_BRAKE, MOVE_BACKWARD, MOVE_TURN), garantissant un comportement déterministe et robuste.



https://github.com/user-attachments/assets/55dbaec2-2ef9-4583-8b24-4435a5f01540



## Conclusion

Ce projet a permis de mettre en pratique l’ensemble de la chaîne de conception d’un système embarqué autonome, depuis le routage d’un PCB jusqu’à la programmation temps réel sous FreeRTOS.  
Malgré certaines fonctionnalités incomplètes (Bluetooth, LIDAR), le robot démontre un comportement fiable et sécurisé face aux événements critiques. Les défis rencontrés ont été formateurs et ont mis en évidence l’importance de l’architecture logicielle et des choix matériels en amont dans un système embarqué réel.
