import rclpy
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from rclpy.node import Node

from std_msgs.msg import String


class GoalPredictor(Node):

    def __init__(self):
        super().__init__('goal_predictor')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        # Define the destination set
        self.D = np.array([[10, 1], [10, 8], [20, 20]])  # Example destinations (x, y)

        # Gaussian distribution parameters
        self.sigma_phi = 0.1
        
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        t_max = 50
        self.dynamic_visualization(t_max, self.D)
    
    def pedestrian_state(self, t):
        # Simulated pedestrian movement: moving in a sinusoidal pattern
        pos = np.array([t, t + np.sin(t)])  # Position
        vel = np.array([1, np.cos(t)])      # Velocity
        return pos, vel

    # Function to compute the angle between velocity and the vector to the destination
    def compute_angle(self, pos, vel, dest):
        direction_to_dest = dest - pos
        angle = np.arccos(np.dot(vel, direction_to_dest) / (np.linalg.norm(vel) * np.linalg.norm(direction_to_dest)))
        return angle

    # Compute the probability P(sp(t+Δt)|Dm)
    def compute_probability(self, pos, vel, dest, sigma):
        angle = self.compute_angle(pos, vel, dest)
        prob = norm.pdf(angle, 0, sigma)  # Gaussian distribution
        return prob

    # Bayesian classifier for destination prediction
    def predict_destination(self, t, D, w=5):
        # Store recent pedestrian states
        recent_states = []
        
        for i in range(w):
            pos, vel = self.pedestrian_state(t - i)
            recent_states.append((pos, vel))
        
        destination_probs = []
        for dest in D:
            # Compute joint probability for each destination
            joint_prob = 1
            for pos, vel in recent_states:
                joint_prob *= self.compute_probability(pos, vel, dest, self.sigma_phi)
            
            destination_probs.append(joint_prob)
        
        # Normalize probabilities
        destination_probs = np.array(destination_probs) / np.sum(destination_probs)
        
        # Return the most likely destination
        return D[np.argmax(destination_probs)], destination_probs

    # Visualization of navigation and dynamic goal prediction
    def dynamic_visualization(self, t_max, D):
        # Initialize figure
        plt.ion()  # Enable interactive mode
        fig, ax = plt.subplots()
        
        for t in range(t_max):
            pos, vel = self.pedestrian_state(t)
            pred_dest, destination_probs = self.predict_destination(t, D)
            
            # Clear previous plot
            ax.clear()
            
            # Plot pedestrian and velocity
            ax.quiver(pos[0], pos[1], vel[0], vel[1], color='r', scale=10, label='Pedestrian Velocity')
            
            # Plot all destinations
            ax.scatter(D[:, 0], D[:, 1], c='blue', label='Destinations')
            
            # Highlight predicted destination
            ax.scatter(pred_dest[0], pred_dest[1], c='green', label='Predicted Destination', marker='X', s=100)
            
            # Add probability annotations for each destination
            for i, (dest, prob) in enumerate(zip(D, destination_probs)):
                ax.text(dest[0], dest[1] + 0.5, f'P(D{i+1}): {prob:.2f}', color='black', fontsize=10)
            
            # Set plot limits
            ax.set_xlim(0, 25)
            ax.set_ylim(0, 20)
            
            # Add title and legend
            ax.set_title(f"Time: {t} | Predicted Destination: {pred_dest}")
            ax.legend()
            
            # Update the plot
            plt.draw()
            plt.pause(0.1)
        
        plt.ioff()  # Disable interactive mode
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    goal_predictor = GoalPredictor()

    rclpy.spin(goal_predictor)
    goal_predictor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()