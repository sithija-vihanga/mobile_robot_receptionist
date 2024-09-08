import rclpy
import numpy as np
import matplotlib.pyplot    as plt
import matplotlib.cm        as cm
from rclpy.node     import Node
from std_msgs.msg   import Float32MultiArray

class CrowdFlowPublisher(Node):

    def __init__(self):
        super().__init__('crowd_flow_publisher')
        self.publisher_     = self.create_publisher(Float32MultiArray, 'map_data', 10)
        timer_period        = 0.2  # seconds
        self.timer          = self.create_timer(timer_period, self.timer_callback)

        self.num_agents         = 10
        self.goal_radius        = 0.5
        self.time_step          = 0.05
        self.agent_radius       = 0.4
        self.max_speed          = 1.0
        self.num_obstacles      = 20
        self.obstacle_radius    = 0.4
        self.avoidance_strength = 0.9 

        np.random.seed(10)
        self.positions  = np.random.rand(self.num_agents, 2) * 10
        self.goals      = np.random.rand(self.num_agents, 2) * 10
        self.obstacles  = np.random.rand(self.num_obstacles, 2) * 10

        self.mapData    = np.zeros(2 + 4*self.num_agents)
        self.mapData[0] = self.num_agents
        self.mapData[1] = self.num_obstacles
        
        # Store paths for each agent
        self.paths = [self.positions[i:i+1].tolist() for i in range(self.num_agents)]

        # Visualization setup
        plt.ion()
        fig, ax = plt.subplots()
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)

        colors = cm.get_cmap('tab10', self.num_agents)  # Get 10 distinct colors

        self.agent_circles      = [self.create_circle(self.positions[i, 0], self.positions[i, 1], self.agent_radius, colors(i)) for i in range(self.num_agents)]
      
        goals = []
        for i in range(self.num_agents):
            goal = plt.Circle(self.goals[i], self.goal_radius, color='green')
            goals.append(goal)
            ax.add_patch(goals[i])
        
        obstacles = []
        for j in range(self.num_obstacles):
            obstacle = plt.Circle(self.obstacles[j], self.obstacle_radius, color='black')
            obstacles.append(obstacle)
            ax.add_patch(obstacles[j])
            

        # Add labels inside the circles
        self.goal_labels    = [ax.text(self.goals[i, 0], self.goals[i, 1], str(i), color='black', ha='center', va='center') for i in range(self.num_agents)]
        self.agent_labels   = [ax.text(self.positions[i, 0], self.positions[i, 1], str(i), color='black', ha='center', va='center') for i in range(self.num_agents)]

        # Initialize scatter plots 
        self.path_dots = [ax.scatter([], [], c=colors(i), s=10, alpha=0.8) for i in range(self.num_agents)]


    def timer_callback(self):
        for j in range(self.num_agents):
            self.mapData[2*j+2:2*j+4] = self.paths[j][-1] 
        self.mapData[2+2*self.num_agents:] = self.goals.flatten()
        msg = Float32MultiArray()
        msg.data = tuple(self.mapData)
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.crowd_flow()

    def create_circle(self, x, y, radius, color):
        return plt.Circle((x, y), radius, color=color, fill=False, lw=2)
    
    def check_obstacle_collision(self, position, obstacle, obstacle_radius):
        distance = np.linalg.norm(position - obstacle)
        return distance < obstacle_radius*2   # obstacle_radius + agent_radius

    # Function to adjust the agent's direction to avoid obstacles
    def avoid_obstacles(self, position, direction, obstacles):
        avoidance_vector = np.zeros(2)
        for obstacle in obstacles:
            if self.check_obstacle_collision(position, obstacle, self.obstacle_radius):
                # Compute the vector away from the obstacle
                avoidance_dir = position - obstacle
                avoidance_distance = np.linalg.norm(avoidance_dir)
                if avoidance_distance > 0:
                    avoidance_dir /= avoidance_distance  # Normalize | Unit vector
                    # Add to the avoidance vector with weight proportional to proximity
                    avoidance_vector += avoidance_dir * (1 / avoidance_distance) * self.avoidance_strength

        # Modify the direction based on the avoidance vector
        adjusted_direction = direction + avoidance_vector
        if np.linalg.norm(adjusted_direction) > 0:
            adjusted_direction /= np.linalg.norm(adjusted_direction)  # Normalize
        
        return adjusted_direction

    def update_positions(self, positions, goals):
        for i in range(self.num_agents):
            # Vector towards the goal
            direction = goals[i] - positions[i]
            distance_to_goal = np.linalg.norm(direction)
            
            # Normalize direction
            if distance_to_goal > 0:
                direction /= distance_to_goal
            
            # Adjust the direction to avoid obstacles
            direction = self.avoid_obstacles(positions[i], direction, self.obstacles)
            
            # Move towards the goal (or adjusted direction if avoiding obstacles)
            new_position = positions[i] + direction * self.max_speed * self.time_step
            
            # Check for collisions with other agents
            for j in range(self.num_agents):
                if i != j:
                    distance = np.linalg.norm(new_position - positions[j])
                    if distance < self.agent_radius * 2:  # Simple collision detection
                        # Simple collision avoidance: move away
                        avoidance_direction = new_position - positions[j]
                        avoidance_direction /= np.linalg.norm(avoidance_direction)
                        new_position += avoidance_direction * (self.agent_radius * 2 - distance)
            
            # Update position
            self.positions[i] = new_position
            self.paths[i].append(new_position.tolist())  # Record the new position in the path
    
    def crowd_flow(self):
        self.update_positions(self.positions, self.goals)
        
        # Update agent positions in the plot
        for i in range(self.num_agents):
            self.agent_circles[i].center = self.positions[i]
            self.agent_labels[i].set_position(self.positions[i])

            # Update the path using small dots
            path_positions = np.array(self.paths[i])
            num_path_dots = len(path_positions)
            fade = np.linspace(0, 1, num_path_dots)  # Gradual fade effect
            self.path_dots[i].set_offsets(path_positions)
            self.path_dots[i].set_alpha(fade)

        # Redraw the plot
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    crowd_flow_publisher = CrowdFlowPublisher()
    rclpy.spin(crowd_flow_publisher)
    crowd_flow_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()