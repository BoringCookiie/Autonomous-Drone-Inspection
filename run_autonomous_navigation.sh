#!/bin/bash
# run_autonomous_navigation.sh - Orchestrateur de mission autonome
set -e

# Configuration
CONTAINER="uas_sim"
GOAL_X=${1:-8.0}
GOAL_Y=${2:-0.5}
GOAL_Z=${3:-1.5}

# 1. Vérification de l'environnement
if ! docker info > /dev/null 2>&1; then
    echo "Erreur: Docker n'est pas lancé."
    exit 1
fi

# 2. Lancement du conteneur si nécessaire
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || \
   ! docker exec "$CONTAINER" test -f /home/uas/docker/bootstrap_px4.sh 2>/dev/null; then
    echo "[Master] Démarrage des conteneurs via Docker Compose..."
    docker compose --project-directory . -f docker/docker-compose.yml --profile sim up -d --force-recreate sim_stack
    echo "[Master] Attente du démarrage du conteneur..."
    sleep 10
fi

# 3. Lancement de la simulation (via le script existant)
echo "[Master] Lancement de la simulation (obstacle_maze)..."
PX4_GZ_WORLD=obstacle_maze PX4_GZ_MODEL_TARGET=gz_x500_depth ./docker/launch_obstacle_stack.sh --no-attach --no-fly

# 4. Attente de la disponibilité de MAVROS
echo "[Master] Attente de la télémétrie (MAVROS - peut prendre plusieurs minutes lors du premier lancement)..."
READY=0
# Augmenté à 300s (5 min) pour laisser le temps à la compilation PX4/Gazebo si nécessaire
for i in {1..300}; do
    if docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null" | grep -q "/uas1/state"; then
        echo -e "\n[Master] Topic /uas1/state trouvé."
        READY=1
        break
    fi
    echo -n "."
    sleep 1
done

if [ -z "${READY:-}" ] || [ "$READY" -eq 0 ]; then
    echo -e "\nErreur: MAVROS n'a pas démarré à temps."
    echo "[Master] Vérifiez l'état de la simulation dans tmux: 'tmux attach -t uas_obstacle'"
    echo "[Master] Fenêtre 0 (px4_gz) : Compilation ou erreur Gazebo"
    echo "[Master] Fenêtre 1 (mavros) : Erreur de connexion MAVLink"
    exit 1
fi

# 4.1 Attendre que les services MAVROS soient disponibles
echo "[Master] Attente des services MAVROS..."
SERVICES_READY=0
for i in {1..60}; do
    if docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && ros2 service list 2>/dev/null" | grep -q "/uas1/param/set"; then
        echo -e "\n[Master] Services MAVROS disponibles."
        SERVICES_READY=1
        break
    fi
    echo -n "."
    sleep 1
done

if [ "$SERVICES_READY" -eq 0 ]; then
    echo -e "\nErreur: Les services MAVROS n'ont pas démarré à temps."
    echo "[Master] Services disponibles:"
    docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && ros2 service list" || true
    exit 1
fi

# 4.5 Configuration des paramètres PX4 pour le SITL
echo "[Master] Configuration des paramètres PX4..."
docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && {
    echo '[Config] COM_ARM_WO_GPS=1'
    timeout 10 ros2 service call /uas1/param/set mavros_msgs/srv/ParamSet '{param_id: \"COM_ARM_WO_GPS\", value: {integer: 1}}' 2>/dev/null || true
    
    echo '[Config] COM_RC_IN_MODE=4'
    timeout 10 ros2 service call /uas1/param/set mavros_msgs/srv/ParamSet '{param_id: \"COM_RC_IN_MODE\", value: {integer: 4}}' 2>/dev/null || true
    
    echo '[Config] NAV_RCL_ACT=0'
    timeout 10 ros2 service call /uas1/param/set mavros_msgs/srv/ParamSet '{param_id: \"NAV_RCL_ACT\", value: {integer: 0}}' 2>/dev/null || true
    
    echo '[Config] EKF2_GPS_CHECK=0'
    timeout 10 ros2 service call /uas1/param/set mavros_msgs/srv/ParamSet '{param_id: \"EKF2_GPS_CHECK\", value: {integer: 0}}' 2>/dev/null || true
} && echo '[Master] Configuration complétée.' || echo '[Warning] Configuration partielle - continuant...'"

# 5. Lancement de la pile de navigation
echo "[Master] Lancement de la pile de navigation (RTAB-Map, OctoMap, Planner, Follower)..."
docker exec -d $CONTAINER bash -c "source /opt/ros/humble/setup.bash && export PYTHONPATH=\$PYTHONPATH:/home/uas/scripts && ros2 launch /home/uas/scripts/navigation/navigation_launch.py"

# Build and launch the inspection package against the migrated runtime.
echo "[Master] Construction de la pile d'inspection..."
docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && /home/uas/scripts/build_workspace.sh"
DETECTOR_BACKEND=${DETECTOR_BACKEND:-rag_vlm}
FLIGHT_STRATEGY=${FLIGHT_STRATEGY:-revisit}
echo "[Master] Lancement de l'inspection (${DETECTOR_BACKEND}, ${FLIGHT_STRATEGY})..."
docker exec -d $CONTAINER bash -c "source /opt/ros/humble/setup.bash && source /home/uas/ros2_ws/install/setup.bash && ros2 launch uas_earthen_inspection inspection_pipeline.launch.py detector_backend:=${DETECTOR_BACKEND} flight_strategy:=${FLIGHT_STRATEGY}"

# Attente pour laisser l'OctoMap et RTAB-Map se construire
echo "[Master] Attente de l'initialisation de la pile de navigation (30s)..."
for i in {1..30}; do
    echo -n "."
    sleep 1
done
echo -e "\n[Master] Vérification des nœuds de navigation..."

# Vérification que les nœuds cruciaux sont bien lancés
NODES_OK=1
for node in "/rtabmap" "/octomap_server" "/planner_3d" "/path_follower"; do
    if ! docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && ros2 node list" | grep -q $node; then
        echo "[Error] Le nœud $node n'est pas démarré."
        NODES_OK=0
    fi
done

if [ "$NODES_OK" -eq 0 ]; then
    echo "[Master] Erreur: Certains nœuds n'ont pas démarré. Vérifiez les logs avec 'docker logs $CONTAINER'."
    exit 1
fi
echo "[Master] Tous les nœuds de navigation sont opérationnels."

# 5.5 Armer le drone
echo "[Master] Armement du drone..."
docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && timeout 10 ros2 service call /uas1/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'" || true
sleep 2

# 6. Envoi de l'objectif
echo "[Master] Envoi de l'objectif : X=$GOAL_X, Y=$GOAL_Y, Z=$GOAL_Z"
if docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && python3 /home/uas/scripts/navigation/send_goal.py $GOAL_X $GOAL_Y $GOAL_Z"; then
    echo "[Master] Objectif envoyé avec succès. Le drone devrait commencer sa mission."
else
    echo "[Master] Erreur lors de l'envoi de l'objectif."
    exit 1
fi

echo "----------------------------------------------------------------"
echo "MISSION EN COURS. Appuyez sur Ctrl+C pour arrêter et générer la vidéo."
echo "----------------------------------------------------------------"

# Nettoyage et analyse lors de l'arrêt
cleanup() {
    echo -e "\n[Master] Arrêt de la mission..."
    
    # Arrêt des nœuds de navigation
    docker exec $CONTAINER bash -c "pkill -f 'planner_3d' || true; pkill -f 'path_follower' || true"
    
    echo "[Master] Génération de la vidéo à partir du dernier rosbag..."
    # On laisse un peu de temps pour fermer le rosbag
    sleep 2
    docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && python3 /home/uas/scripts/analyze_rosbags.py --bag latest --export-video"
    
    # Récupération du chemin de la vidéo
    VIDEO_PATH=$(find rosbags -name "camera.mp4" | sort | tail -n 1)
    if [ -n "$VIDEO_PATH" ]; then
        echo "[Master] Succès ! Vidéo disponible ici : $VIDEO_PATH"
    else
        echo "[Master] Vidéo non trouvée. Vérifiez les logs de analyze_rosbags.py."
    fi
    
    # Arrêt de la simulation tmux
    tmux kill-session -t uas_obstacle 2>/dev/null || true
    exit
}

trap cleanup SIGINT

# Boucle de maintien
while true; do sleep 1; done
