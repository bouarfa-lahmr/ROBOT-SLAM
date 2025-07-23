# 🤖 SLAM Implementation in Labyrinthine Environment

A comprehensive autonomous navigation system for mobile robots using ROS 2, featuring SLAM mapping, AMCL localization, and comparative analysis of path planning algorithms (Dijkstra, A*, BFS, and Theta*) in complex maze environments.

## 🎯 Project Overview

This project implements a complete autonomous navigation system based on ROS 2, integrating a modular and adaptable approach. The robot can build a map of its environment, localize itself precisely using AMCL method, and navigate to targets while dynamically avoiding obstacles. Beyond the classic SLAM and navigation implementation with Nav2, this work includes an in-depth comparative analysis of several path planning algorithms.

### Key Features

- **Complete SLAM Implementation**: Real-time mapping using slam_toolbox
- **Precise Localization**: AMCL-based probabilistic localization
- **Autonomous Navigation**: Nav2 stack integration for path planning and obstacle avoidance
- **Algorithm Comparison**: Comprehensive analysis of Dijkstra, A*, BFS, and Theta* algorithms
- **ROS 2 Integration**: Full implementation using ROS 2 Humble Hawksbill
- **Simulation Environment**: Complex 3D maze testing in Gazebo

## 🎓 Academic Context

- **Institution**: Euro-Mediterranean University of Fez (EuroMed)
- **School**: Digital Engineering and Artificial Intelligence (EIDIA)
- **Department**: Robotics and Cobotics
- **Supervisors**: Pr. Badr EL KARI, Pr. Hiba SEKKAT
- **Team**: BOUARFA LAHMAR, Mohamed BOFARHA, Hiba MOUHSINE

## 🏗️ System Architecture

### Hardware Platform
- **Robot**: TurtleBot3 Waffle
- **Sensors**: 360° LiDAR, IMU, Wheel Encoders
- **Simulation**: Gazebo 3D environment
- **Visualization**: RViz for real-time monitoring

### Software Stack
- **OS**: Ubuntu 22.04 LTS
- **Middleware**: ROS 2 Humble Hawksbill
- **SLAM**: slam_toolbox
- **Navigation**: Nav2 stack
- **Localization**: AMCL (Adaptive Monte Carlo Localization)
- **Programming**: Python, C++

## 🚀 Installation & Setup

### Prerequisites

```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*

# Install navigation and SLAM packages
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
```

### Environment Setup

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/slam-navigation-labyrinth.git
   cd slam-navigation-labyrinth
   ```

2. **Build the workspace**
   ```bash
   colcon build
   source install/setup.bash
   ```

3. **Set TurtleBot3 model**
   ```bash
   export TURTLEBOT3_MODEL=waffle
   echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
   ```

## 📁 Project Structure

```
slam-navigation-labyrinth/
│
├── src/
│   ├── slam_navigation/
│   │   ├── launch/
│   │   │   ├── slam_launch.py
│   │   │   ├── navigation_launch.py
│   │   │   └── full_system_launch.py
│   │   ├── config/
│   │   │   ├── nav2_params.yaml
│   │   │   └── slam_params.yaml
│   │   └── maps/
│   │       ├── maze_map.pgm
│   │       └── maze_map.yaml
│   │
│   └── path_planning_analysis/
│       ├── algorithms/
│       │   ├── dijkstra.py
│       │   ├── astar.py
│       │   ├── bfs.py
│       │   └── theta_star.py
│       ├── maze_generator.py
│       ├── performance_analyzer.py
│       └── visualization.py
│
├── worlds/
│   └── maze_world.world
│
├── docs/
│   ├── algorithm_comparison.md
│   ├── slam_implementation.md
│   └── navigation_setup.md
│
├── results/
│   ├── performance_metrics.csv
│   ├── algorithm_comparison_charts.png
│   └── navigation_videos/
│
└── README.md
```

## 🗺️ SLAM Implementation

### Map Generation Process

1. **Launch SLAM system**
   ```bash
   ros2 launch slam_navigation slam_launch.py
   ```

2. **Manual exploration**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

3. **Save generated map**
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/maps/maze_map
   ```

### SLAM Configuration

The SLAM system uses the following key parameters:

```yaml
# slam_params.yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None
    mode: mapping
    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 20.0
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
```

## 🧭 Navigation System

### AMCL Localization

The Adaptive Monte Carlo Localization provides robust pose estimation:

```python
# Initial pose publisher for AMCL
def publish_initial_pose():
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = self.get_clock().now().to_msg()
    
    # Set initial position and orientation
    initial_pose.pose.pose.position.x = 0.0
    initial_pose.pose.pose.position.y = 0.0
    initial_pose.pose.pose.orientation.w = 1.0
    
    self.initial_pose_pub.publish(initial_pose)
```

### Nav2 Stack Integration

Launch complete navigation system:

```bash
ros2 launch slam_navigation navigation_launch.py map:=maze_map.yaml
```

The navigation stack includes:
- **Map Server**: Loads pre-generated maps
- **AMCL**: Probabilistic localization
- **Planner Server**: Global path planning
- **Controller Server**: Local trajectory following
- **BT Navigator**: Behavior tree execution
- **Recovery Server**: Error handling and recovery behaviors

## 📊 Path Planning Algorithm Analysis

### Test Environment

The comparative analysis uses a 2D maze environment with the following specifications:

- **Dimensions**: 20m × 20m
- **Wall Height**: 2.5m
- **Wall Thickness**: 0.15m
- **Complexity**: Multiple dead ends, narrow corridors, and bifurcations

### Algorithm Implementations

#### 1. Breadth-First Search (BFS)
```python
def bfs(maze, start, goal):
    queue = deque([start])
    visited = set([start])
    parent = {start: None}
    
    while queue:
        current = queue.popleft()
        
        if current == goal:
            return reconstruct_path(parent, start, goal)
        
        for neighbor in get_neighbors(maze, current):
            if neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = current
                queue.append(neighbor)
    
    return None
```

#### 2. Dijkstra's Algorithm
```python
def dijkstra(maze, start, goal):
    distances = {start: 0}
    pq = [(0, start)]
    visited = set()
    parent = {start: None}
    
    while pq:
        current_dist, current = heapq.heappop(pq)
        
        if current in visited:
            continue
            
        visited.add(current)
        
        if current == goal:
            return reconstruct_path(parent, start, goal)
        
        for neighbor in get_neighbors(maze, current):
            distance = current_dist + get_cost(current, neighbor)
            
            if neighbor not in distances or distance < distances[neighbor]:
                distances[neighbor] = distance
                parent[neighbor] = current
                heapq.heappush(pq, (distance, neighbor))
    
    return None
```

#### 3. A* Algorithm
```python
def astar(maze, start, goal):
    open_set = [(0, start)]
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    parent = {start: None}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            return reconstruct_path(parent, start, goal)
        
        for neighbor in get_neighbors(maze, current):
            tentative_g = g_score[current] + get_cost(current, neighbor)
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                parent[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None
```

#### 4. Theta* Algorithm
```python
def theta_star(maze, start, goal):
    open_set = [(0, start)]
    g_score = {start: 0}
    parent = {start: None}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            return reconstruct_path(parent, start, goal)
        
        for neighbor in get_neighbors(maze, current):
            if line_of_sight(maze, parent[current], neighbor):
                # Path 2: Direct line from parent to neighbor
                if parent[current] is not None:
                    tentative_g = g_score[parent[current]] + euclidean_distance(parent[current], neighbor)
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        parent[neighbor] = parent[current]
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score, neighbor))
            else:
                # Path 1: Standard grid-based path
                tentative_g = g_score[current] + euclidean_distance(current, neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    parent[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
    
    return None
```

## 📈 Performance Analysis Results

### Comparative Metrics

| Algorithm | Path Length | Nodes Explored | Execution Time (s) | Efficiency Ratio |
|-----------|-------------|----------------|-------------------|------------------|
| **BFS** | 791 | 76,502 | 0.090 | 0.0103 |
| **Dijkstra** | 791 | 76,350 | 0.307 | 0.0104 |
| **A*** | 791 | 33,045 | 0.249 | 0.0239 |
| **Theta*** | 29 | 43,773 | 1.367 | 0.0007 |

### Key Findings

#### Algorithm Performance Analysis

1. **BFS (Breadth-First Search)**
   - ✅ Fastest execution time (0.090s)
   - ❌ Explores many nodes without cost consideration
   - ✅ Reliable and correct
   - ❌ Not optimal in weighted graphs

2. **Dijkstra's Algorithm**
   - ✅ Guarantees optimal path
   - ❌ Explores extensively without heuristic guidance
   - ❌ Slower than BFS and A*
   - ✅ Suitable for critical applications requiring guaranteed optimality

3. **A* Algorithm**
   - ✅ Excellent balance between performance and solution quality
   - ✅ 57% fewer nodes explored compared to Dijkstra
   - ✅ Maintains optimality with heuristic guidance
   - ✅ Best overall performance for most scenarios

4. **Theta* Algorithm**
   - ✅ Significantly shorter path length (29 vs 791)
   - ✅ Allows direct line-of-sight movements
   - ❌ Highest computational cost (1.367s)
   - ✅ Optimal for applications prioritizing path length over computation time

### Visualization Results

#### Execution Time Comparison
The performance analysis shows clear differences in computational efficiency:

- **BFS**: 0.090s (fastest)
- **A***: 0.249s (good balance)
- **Dijkstra**: 0.307s (slower due to exhaustive search)
- **Theta***: 1.367s (slowest due to line-of-sight calculations)

#### Node Exploration Efficiency
A* demonstrates superior exploration efficiency:

- **A***: 33,045 nodes (most efficient)
- **Theta***: 43,773 nodes (moderate)
- **Dijkstra**: 76,350 nodes (extensive)
- **BFS**: 76,502 nodes (most extensive)

## 🎮 Usage Instructions

### Running SLAM and Navigation

1. **Start the simulation environment**
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py world:=maze_world.world
   ```

2. **Launch SLAM for mapping**
   ```bash
   ros2 launch slam_navigation slam_launch.py
   ```

3. **Control robot for exploration**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

4. **Save the generated map**
   ```bash
   ros2 run nav2_map_server map_saver_cli -f maze_map
   ```

5. **Launch navigation with saved map**
   ```bash
   ros2 launch slam_navigation navigation_launch.py map:=maze_map.yaml
   ```

### Running Algorithm Comparison

1. **Execute comparative analysis**
   ```bash
   cd src/path_planning_analysis
   python performance_analyzer.py
   ```

2. **Generate visualization**
   ```bash
   python visualization.py --algorithms all --output results/
   ```

3. **View performance metrics**
   ```bash
   cat results/performance_metrics.csv
   ```

## 🔧 Configuration

### Navigation Parameters

Key Nav2 parameters for optimal performance:

```yaml
# nav2_params.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
```

## 🚀 Future Enhancements

### Immediate Improvements
- **Additional Algorithms**: Implementation of RRT*, D* Lite
- **Dynamic Obstacles**: Integration of moving obstacle handling
- **Physical Robot**: Validation on real TurtleBot3 hardware
- **Statistical Analysis**: Extended testing for robust performance metrics
- **Parameter Optimization**: Algorithm tuning for improved performance

### Advanced Features
- **3D Environment**: Extension to complex 3D navigation
- **Multi-Robot**: Coordination of multiple autonomous agents
- **Machine Learning**: Integration of learned path planning strategies
- **Real-time Adaptation**: Dynamic algorithm selection based on environment
- **Cloud Integration**: Distributed processing for complex computations

## 🧪 Testing and Validation

### Test Scenarios

1. **Simple Navigation**: Point-to-point navigation in open areas
2. **Complex Maze**: Navigation through narrow corridors and dead ends
3. **Dynamic Obstacles**: Handling of moving obstacles (future work)
4. **Multi-Goal**: Sequential navigation to multiple waypoints
5. **Recovery Behaviors**: Testing error handling and recovery mechanisms

### Performance Metrics

- **Path Optimality**: Comparison of found path length to theoretical optimum
- **Computational Efficiency**: Execution time and memory usage
- **Robustness**: Success rate across different scenarios
- **Real-time Performance**: Ability to replan during navigation
- **Localization Accuracy**: AMCL pose estimation precision

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Development Guidelines
- Follow ROS 2 coding standards
- Include comprehensive documentation
- Add unit tests for new algorithms
- Test in simulation before hardware deployment
- Update performance benchmarks for new features

## 👥 Team

**Development Team:**
- **BOUARFA LAHMAR** 
- **Mohamed BOFARHA** 
- **Hiba MOUHSINE**

**Academic Supervision:**
- **Pr. Badr EL KARI**
- **Pr. Hiba SEKKAT**

**Institution:** Euro-Mediterranean University of Fez (EuroMed)  
**Department:** Robotics and Cobotics - EIDIA


## 📞 Support & Contact

For all information, questions, help, or collaboration opportunities, feel free to contact the team.
### Additional Support
- Open an issue on GitHub for technical problems
- Check the documentation in `/docs` for detailed guides
- Contact supervisors for academic-related inquiries
- Refer to ROS 2 documentation for framework-specific questions

We're always happy to discuss this project, answer questions about SLAM implementation, path planning algorithms, or collaborate on similar robotics projects!

## 🙏 Acknowledgments

- Euro-Mediterranean University of Fez for providing resources and facilities
- ROS 2 community for excellent robotics middleware
- Open Robotics for TurtleBot3 platform and simulation tools
- Navigation2 team for comprehensive navigation stack
- Academic supervisors for guidance and expertise
- Robotics research community for algorithmic foundations



⭐ **If you found this project helpful, please give it a star!** ⭐

## 🏷️ Tags

`slam` `ros2` `navigation` `path-planning` `autonomous-robot` `turtlebot3` `amcl` `nav2` `dijkstra` `astar` `bfs` `theta-star` `robotics` `gazebo` `rviz`
```
```


