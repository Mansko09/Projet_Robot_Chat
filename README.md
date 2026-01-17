# Projet Robot Chat

**ESE – Projet de groupe**

Ce projet consiste à concevoir et réaliser un **robot mobile à deux roues motrices** capable de participer à un jeu de **chat et souris** contre les robots de nos camarades.

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

   * Liaison **Bluetooth** avec le PC (STM32WB55CEU6)

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

## Conception à réaliser

1. **Électronique**

   * Schéma électrique et PCB personnalisé
   * Routage, fabrication et soudure des composants

2. **Mécanique**

   * Conception du châssis sur **Onshape**
   * Intégration du PCB, moteurs, capteurs et batterie

3. **Logiciel embarqué**

   * Programmation STM32 (CubeIDE)
   * Gestion des rôles (chat/souris)
   * Contrôle moteurs, capteurs et communication Bluetooth


## Architecture générale 

<img width="851" height="541" alt="image" src="https://github.com/user-attachments/assets/b1c18867-8e34-47c1-b6d0-efc3a15b8947" />

## Structure du Code
<img width="805" height="550" alt="image" src="https://github.com/user-attachments/assets/20937b1e-2baa-4d7b-9609-9d910529d608" />

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
  - **Task_Control** : implémentation de la machine à états principale du système.

- **Perception sensorielle anti-chute**  
  Quatre capteurs **ToF VL53L0X** surveillent en permanence les angles du robot afin de détecter la présence de vide.

- **Détection de choc**  
  L’accéléromètre **ADXL343** est configuré en mode *Single Tap*. Un choc mécanique déclenche une interruption matérielle (EXTI) provoquant l’arrêt immédiat du robot et l’allumage de la LED rouge. Un second choc relance le système et active la LED verte.

## Avancement du projet

* [x] Choix des composants et architecture générale
* [x] Schéma et routage PCB
* [x] Impression 3D du robot
* [x] Développement logiciel de base
* [ ] Intégration LIDAR et logique “chat/souris”

