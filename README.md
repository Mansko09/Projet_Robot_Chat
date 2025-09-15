# Projet_Robot_Chat
ESE Projet de groupe

Ce projet consiste à concevoir et réaliser un robot mobile à deux roues motrices capable de participer à un jeu de chat et souris contre les robots de nos camarades.

## Objectifs fonctionnels
Pour atteindre ce comportement, plusieurs fonctions clés doivent être mises en place  :
1. Le robot doit se déplacer :
   - roues
   - moteurs (motoréducteur)
   - driver de moteur : timer
   - capteur (encodeur) : timer
2. Le robot ne doit pas tomber :
   - capteur tof : i2C
3. Le robot doit détecter d'autres robots :
    - LIDAR
5. Le robot doit détecter une collision :
   - Lorsqu'une collision entre deux robots est détectée, ils échangent leurs rôles. (Utilisation d'un accéléromètre)
6. le robot doit communiquer avec le PC en bluetooth (on utilise la STM32WEB55CEU6)
7. Le robot doit s'alimenter en énergie

## Composants principaux
- Microcontrôleur : STM32WB55CEU6 (ARM Cortex-M4, BLE intégré)
- Quartz 16 MHZ (Farnell : 2853935)
- Connecteur SWD/STLink (Farnell : 3226055)
- Moteurs : DfRobot FIT0520 ou FIT0521
- Aliemntation : Batterie NIMH 7.2V 1.3Ah (RS: 777-0377) avec régulateurs 5V MP1475DJ-LF-P et 3.3V  BU33SD5WG-TR
- Actionneurs : 2 moteurs DC + drivers ZXBM5210 (un par roue)
- Capteurs : LIDAR YDLIDAR X2, accéléromètre ADXL343BCCZ-RL
- Connecteurs JST 2.54mm
- LED +R/C
- Boutons poussoirs : : Wurth 430182070816 → SW Push 1P1T NO 6x6mm H9.5mm
- Interrupteur ON/OFF : Wurth 472121020311

## Conception à réaliser : 
1. Electronique :
  - Schéma  électrique et PCB personnalisé
  - Soudure des composants
2. Mécanique :
  - Design robot sous Onshape
3. Logiciel embarqué :
  - Programmation STM32
  - Gestion des rôles, des capteurs et moteurs
  - Communication éventuelle via Bluetooth

## Étapes du projet

[x] Choix des composants et architecture générale
[] Schéma et routage PCB
[] Impression 3D du robot
[] Développement logiciel 
[] Intégration LIDAR et logique “chat/souris”
[] Tests finaux et matchs entre robots 
