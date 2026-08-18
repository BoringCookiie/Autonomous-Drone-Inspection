#!/bin/bash
# health_check.sh - Diagnostic rapide du système DroneIT

set -euo pipefail

# Couleurs
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

CONTAINER="uas_sim"
SESSION="uas_obstacle"

echo -e "${BLUE}═══════════════════════════════════════════${NC}"
echo -e "${BLUE}  DroneIT Health Check${NC}"
echo -e "${BLUE}═══════════════════════════════════════════${NC}\n"

# 1. Docker
echo -e "${BLUE}1. Docker Status${NC}"
if docker info > /dev/null 2>&1; then
    echo -e "   ${GREEN}✓${NC} Docker is running"
else
    echo -e "   ${RED}✗${NC} Docker is NOT running"
    exit 1
fi

# 2. Container
echo -e "\n${BLUE}2. Container Status${NC}"
if docker ps | grep -q $CONTAINER; then
    echo -e "   ${GREEN}✓${NC} Container '$CONTAINER' is running"
    CONTAINER_RUNNING=1
else
    echo -e "   ${YELLOW}⚠${NC}  Container '$CONTAINER' is NOT running"
     echo "   → Run: docker compose --project-directory . -f docker/docker-compose.yml --profile sim up -d --force-recreate sim_stack"
    CONTAINER_RUNNING=0
fi

# 3. tmux session
echo -e "\n${BLUE}3. tmux Session Status${NC}"
if [ $CONTAINER_RUNNING -eq 1 ]; then
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo -e "   ${GREEN}✓${NC} Session '$SESSION' exists"
        
        # Check windows
        WINDOWS=$(tmux list-windows -t "$SESSION" 2>/dev/null | wc -l)
        echo "   → $WINDOWS windows: px4_gz, mavros, camera_bridge, fly/recorder"
    else
        echo -e "   ${YELLOW}⚠${NC}  Session '$SESSION' does NOT exist"
        echo "   → Run: ./docker/launch_obstacle_stack.sh --no-fly"
    fi
else
    echo -e "   ${RED}⚗${NC} Container not running, skipping"
fi

# 4. ROS2 Topics
echo -e "\n${BLUE}4. ROS2 Topics${NC}"
if [ $CONTAINER_RUNNING -eq 1 ]; then
    TOPICS=$(docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null || true" | wc -l)
    
    if [ $TOPICS -gt 0 ]; then
        echo -e "   ${GREEN}✓${NC} ROS2 DDS active ($TOPICS topics)"
        
        # Check critical topics
        if docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null" | grep -q "/uas1/state"; then
            echo -e "   ${GREEN}✓${NC} /uas1/state available"
        else
            echo -e "   ${YELLOW}⚠${NC}  /uas1/state NOT available"
        fi
        
        if docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null" | grep -q "/camera"; then
            echo -e "   ${GREEN}✓${NC} /camera topic available"
            if docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && timeout 6 ros2 topic hz /camera/color/image_raw 2>&1" | grep -q "average rate"; then
                echo -e "   ${GREEN}✓${NC} RGB frames are flowing"
            else
                echo -e "   ${RED}✗${NC} RGB topic exists but no frames are flowing"
            fi
        else
            echo -e "   ${YELLOW}⚠${NC}  /camera topic NOT available"
        fi
    else
        echo -e "   ${RED}✗${NC} No ROS2 topics detected"
    fi
else
    echo -e "   ${RED}✗${NC} Container not running"
fi

# 5. MAVROS Services
echo -e "\n${BLUE}5. MAVROS Services${NC}"
if [ $CONTAINER_RUNNING -eq 1 ]; then
    SERVICES=$(docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && ros2 service list 2>/dev/null || true" | wc -l)
    
    if [ $SERVICES -gt 0 ]; then
        echo -e "   ${GREEN}✓${NC} ROS2 services active ($SERVICES services)"
        
        # Check critical services
        if docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && ros2 service list 2>/dev/null" | grep -q "/uas1/param/set"; then
            echo -e "   ${GREEN}✓${NC} /uas1/param/set available"
        else
            echo -e "   ${RED}✗${NC} /uas1/param/set NOT available"
        fi
        
        if docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && ros2 service list 2>/dev/null" | grep -q "/uas1/cmd/arming"; then
            echo -e "   ${GREEN}✓${NC} /uas1/cmd/arming available"
        else
            echo -e "   ${YELLOW}⚠${NC}  /uas1/cmd/arming NOT available"
        fi
    else
        echo -e "   ${RED}✗${NC} No ROS2 services detected"
    fi
else
    echo -e "   ${RED}✗${NC} Container not running"
fi

# 6. Navigation Stack
echo -e "\n${BLUE}6. Navigation Stack${NC}"
if [ $CONTAINER_RUNNING -eq 1 ]; then
    NODES=$(docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null || true" | wc -l)
    echo "   Nodes running: $NODES"
    
    docker exec $CONTAINER bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null || true" | while read node; do
        if [[ "$node" == *"rtabmap"* ]]; then
            echo -e "   ${GREEN}✓${NC} RTAB-Map running"
        elif [[ "$node" == *"octomap"* ]]; then
            echo -e "   ${GREEN}✓${NC} OctoMap running"
        elif [[ "$node" == *"planner"* ]]; then
            echo -e "   ${GREEN}✓${NC} Planner running"
        elif [[ "$node" == *"follower"* ]]; then
            echo -e "   ${GREEN}✓${NC} Path follower running"
        fi
    done
else
    echo -e "   ${RED}✗${NC} Container not running"
fi

# 7. File structure
echo -e "\n${BLUE}7. Required Files${NC}"
FILES=(
    "run_autonomous_navigation.sh"
    "docker/launch_obstacle_stack.sh"
    "scripts/simulation/run_obstacle_flight.sh"
    "scripts/navigation/navigation_launch.py"
    "scripts/navigation/send_goal.py"
    "scripts/build_workspace.sh"
    "docker/docker-compose.yml"
    "gazebo_simulation/worlds/obstacle_maze.sdf"
)

for file in "${FILES[@]}"; do
    if [ -f "$file" ]; then
        echo -e "   ${GREEN}✓${NC} $file"
    else
        echo -e "   ${RED}✗${NC} $file (MISSING)"
    fi
done

# 8. Docker volumes
echo -e "\n${BLUE}8. Docker Volumes${NC}"
if [ $CONTAINER_RUNNING -eq 1 ]; then
    echo "   Mounted volumes:"
    docker inspect $CONTAINER | grep -A 30 '"Mounts"' | grep -E '"Source"|"Destination"' | sed 's/.*"\(.*\)"/   → \1/'
fi

# 9. System info
echo -e "\n${BLUE}9. System Info${NC}"
echo "   OS: $(uname -s)"
echo "   Kernel: $(uname -r)"
if command -v docker --version >/dev/null 2>&1; then
    echo "   Docker: $(docker --version | awk '{print $3}' | tr -d ',')"
fi
if command -v tmux -V >/dev/null 2>&1; then
    echo "   tmux: $(tmux -V)"
fi

# 10. Quick action recommendations
echo -e "\n${BLUE}10. Recommended Actions${NC}"
ACTIONS=0

if [ $CONTAINER_RUNNING -eq 0 ]; then
    echo -e "   ${YELLOW}→${NC} docker compose --project-directory . -f docker/docker-compose.yml --profile sim up -d --force-recreate sim_stack"
    ACTIONS=1
fi

if [ $CONTAINER_RUNNING -eq 1 ] && ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo -e "   ${YELLOW}→${NC} ./docker/launch_obstacle_stack.sh --no-fly"
    ACTIONS=1
fi

if [ $ACTIONS -eq 0 ]; then
    echo -e "   ${GREEN}✓ System ready! Run: ./run_autonomous_navigation.sh 8.0 0.5 1.5${NC}"
fi

echo -e "\n${BLUE}═══════════════════════════════════════════${NC}\n"
