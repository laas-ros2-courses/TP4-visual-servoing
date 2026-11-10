# TP4 — Asservissement visuel (tête TIAGo)

![distro](https://img.shields.io/badge/ROS2-Jazzy-blue)

## Objective

Asservir la tête de TIAGo (`head_1_joint` = pan, `head_2_joint` = tilt) pour garder un marqueur ArUco centré dans l'image de la caméra tête (`/head_front_camera/image`), en pilotant `head_controller`.

> Statut : une implémentation de référence a été écrite et testée de bout en bout (environnement isolé) — le suivi fonctionne, confirmé en pratique : déplacer le marqueur fait pivoter la tête pour le recentrer. Le dictionnaire ArUco et l'orientation du spawn ci-dessous sont des valeurs confirmées ; le signe des gains proportionnels (section *Nœud de commande*) reste volontairement à déterminer empiriquement par l'étudiant.

## Livrable

À la fin de ce TP, le package `tiago_visual_servoing` doit contenir :
- le nœud `aruco_tracker` (détection du marqueur, publication de sa position pixel),
- le nœud `head_servo` (asservissement de la tête à partir de cette position),
- un fichier de launch unique qui démarre la simulation, le marqueur et les deux nœuds.

Démonstration attendue : déplacer le marqueur ArUco dans Gazebo (clic + glisser) et montrer que la tête de TIAGo le suit en le gardant centré dans l'image, visible dans `rqt_image_view` sur `/aruco_tracker/debug_image`.

## Prerequisites

- Aucun prérequis OpenCV : les notions nécessaires sont expliquées dans la section *Notions OpenCV de base*.

## Setup

**1. Entrer dans l'environnement et builder le workspace.**

Ce TP fournit son propre environnement Nix, déjà configuré avec ROS2/Gazebo Harmonic et les outils de vision nécessaires (OpenCV avec le module ArUco, `cv_bridge`, `rqt_image_view`) :

```bash
nix develop
vcs import src < tiago.repos
./patches/apply.sh
colcon build --symlink-install
source install/setup.bash
```

(Le domaine DDS et la portée de découverte sont déjà fixés par `.envrc` via direnv — voir le README racine du dépôt.)

**2. Vérifier que la caméra tête est bien accessible.**

```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=empty
```

Dans un autre terminal :
```bash
ros2 topic hz /head_front_camera/image
ros2 run rqt_image_view rqt_image_view   # sélectionner /head_front_camera/image
```

**3. Créer le package `tiago_visual_servoing`.**

```bash
cd src/
ros2 pkg create --build-type ament_python \
                --license BSD-3-Clause \
                --maintainer-name <ton-nom> \
                --maintainer-email <ton-email> \
                tiago_visual_servoing
```

Dépendances dans `package.xml` :
```xml
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>trajectory_msgs</depend>
<depend>cv_bridge</depend>
<depend>image_transport</depend>
```

## Notions OpenCV de base

Documentation officielle :
- [OpenCV 4.x — doc générale](https://docs.opencv.org/4.x/)
- [`cv2.aruco.ArucoDetector`](https://docs.opencv.org/4.x/d2/d1a/classcv_1_1aruco_1_1ArucoDetector.html) (référence de la classe utilisée pour la détection)
- [`cv_bridge` (ROS2 Jazzy)](https://docs.ros.org/en/jazzy/p/cv_bridge/)

Ce TP n'a besoin que d'une petite partie d'OpenCV : la détection ArUco fait tout le travail de détection en un seul appel, pas besoin de seuillage de couleur ni de calcul de centre de gravité pixel par pixel. Il faut quand même comprendre quelques notions pour écrire les deux nœuds :

- **Une image OpenCV = un tableau numpy.** Une image couleur arrive comme un tableau de forme `(hauteur, largeur, 3)` (une ligne par ligne de pixels, de haut en bas). Attention à l'ordre des axes : le premier indice est la **ligne** (`y`, verticale), le second la **colonne** (`x`, horizontale) — l'inverse de l'ordre `(x, y)` auquel on pense naturellement pour une position dans l'image.
- **BGR, pas RGB.** OpenCV stocke les couleurs dans l'ordre Bleu-Vert-Rouge par convention historique, pas Rouge-Vert-Bleu. Ça n'a pas d'impact pour la détection ArUco (qui travaille en niveaux de gris en interne), mais ça compte si on affiche ou sauvegarde une image et que les couleurs paraissent inversées.
- **`cv_bridge` fait le pont ROS ↔ OpenCV.** Un message `sensor_msgs/Image` n'est qu'un tableau d'octets bruts avec un encodage déclaré (ex. `"bgr8"`). `CvBridge().imgmsg_to_cv2(msg, "bgr8")` le convertit en tableau numpy exploitable par OpenCV ; `CvBridge().cv2_to_imgmsg(image, "bgr8")` fait l'inverse (utile pour republier l'image annotée de debug).
- **`sensor_msgs/CameraInfo` donne le centre optique réel.** Le champ `k` (matrice intrinsèque 3×3 à plat) contient `c_x = k[2]`, `c_y = k[5]` : le vrai centre de l'image, qui peut différer légèrement de `largeur/2, hauteur/2` selon le capteur. C'est ce qu'on utilise comme référence pour l'erreur, plutôt qu'une valeur codée en dur.
- **Détection ArUco = une seule fonction.** `cv2.aruco.ArucoDetector(dictionnaire, params).detectMarkers(image)` renvoie trois valeurs :
  - `corners` : une liste (une par marqueur détecté) de tableaux de forme `(1, 4, 2)` — les 4 coins du marqueur en pixels, dans l'ordre (haut-gauche, haut-droite, bas-droite, bas-gauche).
  - `ids` : un tableau `(N, 1)` avec l'identifiant de chaque marqueur détecté (utile si plusieurs marqueurs différents sont dans le champ de la caméra).
  - `rejected` : des candidats qui ressemblaient à un marqueur mais n'ont pas été validés (pas utile ici).

  Le centre pixel d'un marqueur détecté est simplement la moyenne des 4 coins : `corners[i][0].mean(axis=0)` → `(u, v)`.

## Le marqueur ArUco

Pas besoin de fabriquer un marqueur : le paquet `br2_gazebo_worlds` (récupéré par `tiago.repos`, voir Setup) fournit déjà une collection de tags ArUco prêts à l'emploi sous forme de modèles Gazebo texturés — `aruco_tag_30`, `aruco_tag_40`, ..., jusqu'à `aruco_tag_124` (le nombre dans le nom est l'ID du marqueur). On utilisera `aruco_tag_60` à titre d'exemple ; n'importe quel autre ID de la liste convient aussi.

Faire apparaître le marqueur dans le monde Gazebo. Les modèles `aruco_tag_*` sont à plat par défaut : `-P -1.5708` (pitch −90°) les relève pour qu'ils fassent face au robot plutôt que de rester couchés au sol, invisibles pour la caméra tête.

```bash
ros2 pkg prefix br2_gazebo_worlds   # localiser le paquet installé (install/br2_gazebo_worlds)
ros2 run ros_gz_sim create -file install/br2_gazebo_worlds/share/br2_gazebo_worlds/models/aruco_tag_60/model.sdf -name aruco_60 -x 1.5 -y 0.0 -z 1.0 -P -1.5708
```

Dictionnaire ArUco utilisé par ces tags : `cv2.aruco.DICT_4X4_250` (confirmé pour `aruco_tag_30` à `aruco_tag_110`) — exception : `aruco_tag_124` utilise `DICT_6X6_250`.

Une version imprimable du même marqueur (`DICT_4X4_250`, ID 60, avec bordure blanche) est fournie dans [`markers/aruco_tag_60.png`](./markers/aruco_tag_60.png) — pas utile pour la simulation (le modèle Gazebo suffit), mais nécessaire pour le bonus *tester sur le robot réel* ci-dessous.

## Nœud de détection (`aruco_tracker`)

Fichier `tiago_visual_servoing/aruco_tracker.py`. Squelette à compléter (les `TODO` sont à écrire) :

```python
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

ARUCO_DICTIONARY = cv2.aruco.DICT_4X4_250  # aruco_tag_30..110 ; aruco_tag_124 -> DICT_6X6_250


class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        self.bridge = CvBridge()
        dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY)
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

        self.target_pub = self.create_publisher(PointStamped, '/aruco_tracker/target_pixel', 10)
        self.debug_pub = self.create_publisher(Image, '/aruco_tracker/debug_image', 10)
        self.create_subscription(Image, '/head_front_camera/image', self.image_callback, 10)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        corners, ids, rejected = self.detector.detectMarkers(image)

        # TODO : si `ids` n'est pas vide, prendre le premier marqueur détecté,
        # calculer son centre pixel (corners[0][0].mean(axis=0) -> (u, v)) et
        # le publier sur self.target_pub (PointStamped, x=u, y=v). Si `ids`
        # est vide, ne rien publier sur target_pixel (voir section suivante).

        # TODO : republier une image annotée sur self.debug_pub
        # (cv2.aruco.drawDetectedMarkers(...) + un cercle sur le centre
        # calculé) pour la visualiser dans rqt_image_view pendant les réglages.


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Nœud de commande (`head_servo`)

Fichier `tiago_visual_servoing/head_servo.py`. Squelette à compléter :

```python
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, JointState
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

PAN_JOINT = 'head_1_joint'
TILT_JOINT = 'head_2_joint'
PAN_LIMITS = (math.radians(-75.0), math.radians(75.0))
TILT_LIMITS = (math.radians(-60.0), math.radians(45.0))

KP_PAN = ...    # TODO : choisir signe + magnitude (voir note sous le code)
KP_TILT = ...
DEADZONE_PX = 6.0
MAX_STEP_RAD = math.radians(3.0)


class HeadServo(Node):
    def __init__(self):
        super().__init__('head_servo')
        self.cx = None
        self.cy = None
        self.pan = 0.0
        self.tilt = 0.0

        self.create_subscription(CameraInfo, '/head_front_camera/camera_info', self.camera_info_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(PointStamped, '/aruco_tracker/target_pixel', self.target_cb, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)

    def camera_info_cb(self, msg):
        # c_x = k[2], c_y = k[5] : le vrai centre optique, plutôt qu'une
        # valeur codée en dur (320/240).
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def joint_state_cb(self, msg):
        # `head_controller` est en command_interfaces: [position] (pas de
        # vitesse) : on a besoin de connaître la position courante pour lui
        # envoyer des incréments de position, plutôt que des vitesses.
        for name, position in zip(msg.name, msg.position):
            if name == PAN_JOINT:
                self.pan = position
            elif name == TILT_JOINT:
                self.tilt = position

    def target_cb(self, msg):
        if self.cx is None:
            return

        du = msg.point.x - self.cx
        dv = msg.point.y - self.cy

        # TODO : zone morte — ne rien faire si |du| et |dv| sont déjà
        # petites (DEADZONE_PX).

        # TODO : incrément d'angle proportionnel à l'erreur
        # (dpan = KP_PAN * du, dtilt = KP_TILT * dv), saturé à ±MAX_STEP_RAD,
        # ajouté à self.pan / self.tilt, puis clampé à PAN_LIMITS / TILT_LIMITS.

        # TODO : publier le résultat via self.send_trajectory(pan, tilt).

    def send_trajectory(self, pan, tilt):
        traj = JointTrajectory()
        traj.joint_names = [PAN_JOINT, TILT_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = Duration(sec=0, nanosec=150_000_000)
        traj.points = [point]
        self.traj_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = HeadServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

> Signe des gains à valider empiriquement : il faut que le marqueur se déplace *vers* le centre, pas l'inverse — si ça diverge, inverser le signe d'un des deux gains. Si `target_pixel` n'arrive plus pendant un certain délai (marqueur perdu), ne pas continuer à extrapoler : arrêter d'envoyer de nouvelles commandes (la tête reste sur sa dernière position) plutôt que de deviner où regarder.

## Lancer et tester

**1. Lancer la simulation avec le marqueur.**
```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=empty
ros2 run ros_gz_sim create -file install/br2_gazebo_worlds/share/br2_gazebo_worlds/models/aruco_tag_60/model.sdf -name aruco_60 -x 1.5 -y 0.0 -z 1.0 -P -1.5708
```

**2. Lancer les deux nœuds.**
```bash
ros2 run tiago_visual_servoing aruco_tracker
ros2 run tiago_visual_servoing head_servo
```

**3. Visualiser la détection.**
```bash
ros2 run rqt_image_view rqt_image_view   # sélectionner /aruco_tracker/debug_image
```

**4. Observer le suivi.**

Déplacer le modèle du marqueur dans Gazebo (clic + glisser, ou via `ros2 topic pub` sur une commande de pose si le modèle est articulé) et observer la tête de TIAGo corriger pan/tilt pour le garder centré.

**5. Regrouper dans un launch file.**

Une fois les deux nœuds validés manuellement, écrire `launch/visual_servoing.launch.py` qui lance Gazebo + spawn du marqueur + les deux nœuds dans une seule commande, sur le même principe que les launch files combinés de TP2 (`yahboom_rosmaster_bringup/launch/...`).

## Aller plus loin (bonus)

Le nœud `head_servo` ci-dessus fait du suivi par erreur pixel proportionnelle découplée (pan ← $\Delta u$, tilt ← $\Delta v$) — pas de vraie matrice d'interaction, pas de notion de profondeur. C'est suffisant pour centrer le marqueur, mais ce n'est pas un asservissement visuel complet au sens strict.

Deux directions pour aller plus loin, rendues possibles par le fait qu'un marqueur ArUco donne une pose complète (translation + orientation), pas juste un centroïde 2D :

- **IBVS complet** : utiliser `cv2.aruco.estimatePoseSingleMarkers` (ou `solvePnP` sur les 4 coins) pour obtenir $Z_c$, calculer la vraie matrice d'interaction $L_s$, et comparer la commande obtenue à la version simplifiée.
- **PBVS** : réguler directement l'erreur de pose 3D (translation + orientation) du marqueur dans le repère caméra, sans passer par $L_s$ du tout.
- **Tester sur un TIAGo réel** : le code des deux nœuds ne dépend de rien de spécifique à la simulation (aucun appel Gazebo). Imprimer [`markers/aruco_tag_60.png`](./markers/aruco_tag_60.png) (même dictionnaire/ID qu'en simulation) à une taille raisonnable (par exemple 15×15 cm, bordure blanche conservée) et le présenter devant la caméra tête du robot.

  > ⚠️ À vérifier sur le robot réel avant de lancer les nœuds : les noms de topics (`/head_front_camera/image`, `/head_front_camera/camera_info`, `/head_controller/joint_trajectory`) sont ceux de la simulation TIAGo Harmonic — à confirmer sur le robot physique avec `ros2 topic list` (ils peuvent différer selon la configuration). Se connecter au réseau ROS du robot au préalable (voir le README racine du dépôt sur l'isolation réseau DDS en salle). Superviser les premiers essais : les limites articulaires sont clampées dans le code, mais un mouvement physique reste un mouvement physique — rester prêt à couper (`Ctrl+C` sur `head_servo`).
