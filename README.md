# simple_teleop

Package ros pour la téléopération. Peut publier des messages Twist, Wrench ou Float32 suivant les besoins.
Version de ROS testée : Kinetic

Ce package a été développé pour la commande de multiples actionneurs sur un robot allant d'une commande en force, en vitesse ou encore en position.
Voici les types supportés :
 - Wrench
 - Twist
 - Position (Float32 avec modification additive de la valeur de commande)
 - Int8
 - String

### Préambule
 - le prefix ros devant le nom du repository est là pour permettre de trier facilement les repository. Il est différent du nom du package qui est lui : `simple_teleop`
 - Ce package dispose d'une interface vers le package de gestion de joystick : `Joy`. Dans le launchfile de test cette interface est testée, par conséquent il est nécessaire de télécharger le package `Joy` pour lancer `test.launch`. Ce téléchargement intervient automatiquement si vous suivez les étapes d'installation ci-dessous.

### Installation

1. Déplacez vous au sein d'un workspace ROS et clonez ce repository
`git clone https://github.com/simchanu29/ros_simple_teleop simple_teleop`
2. Déplacez vous à la racine du workspace
`source devel/setup.bash`
3. Résolvez les dépendances : `rosdep install simple_teleop` (cela va probablement lancer l'install du package ROS `joy`)

### Test
Pour tester le package après installation
`roslaunch simple_teleop test.launch`

### Configuration
L'ensemble du mapping des touches se fait via un fichier de config yaml dans le dossier "config"
Le principe est le suivant :

1. On définit les interpréteurs de commande qui vont être créé. Ces interpréteurs peuvent être de différents types exprimés au dessus. On peut alors spécifier les paramètres de ces interpréteurs.

Par exemple, pour commander le robot turtle avec un message twist, on pourrait créer l'interpréteur suivant :
```
twist_turtle:  # nom de l'interpréteur
  type: twist  # type de message envoyé par l'interpréteur
  range_lin: 20.0
  range_ang: 20.0
  precision : 100.0
```

De maniere générale, range c'est l'échelle par laquelle on multiplie la commande qui est comprise dans [-1,1]. La précision c'est le facteur par lequel on multiplie la commande issue de la touche (1 ou -1 habituellement) lors de l'incrémentation de la commande du message. Pour une meilleure compréhension, je conseille de regarder les fichiers fill_xxx.py.

2. On définit les touches associées au topic

Par exemple pour avancer ou reculer :
```
'a':
  called_interpreter: 'twist_turtle'
  value: [1,0,0]

'b':
  called_interpreter: 'twist_turtle'
  value: [-1,0,0]
```
