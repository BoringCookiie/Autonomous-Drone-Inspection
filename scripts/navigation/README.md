# 3D Navigation Stack (A* + OctoMap)

This stack provides autonomous navigation for the drone in Gazebo.

## How to Run
1. Start the simulation stack (with `obstacle_maze.sdf`).
2. Run the navigation script:
   ```bash
   ./scripts/navigation/run_navigation.sh
   ```
3. Send a goal:
   ```bash
   python3 scripts/navigation/send_goal.py 5.0 2.0 1.5
   ```

## Components
- **Planner**: A* algorithm discretized at 0.2m.
- **Follower**: Sends setpoints to MAVROS.
- **SLAM**: RTAB-Map for visual odom and OctoMap.
