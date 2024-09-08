import rclpy
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class GoalPredictor(Node):

    def __init__(self):
        super().__init__('goal_predictor')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'map_data',
            self.listener_callback,
            10)
        self.subscription  

        self.positions = []
        self.goals     = []

        self.pedestrian_pos = []
        self.pedestrian_vel = []
        self.path_buffer = 5

        self.D = np.array([[10, 1], [10, 8], [20, 20]])  # Example destinations (x, y)

        # Gaussian distribution parameters
        self.sigma_phi = 0.1

        #self.timer          = self.create_timer(1, self.visualize)
        
    def listener_callback(self, msg):
        # self.get_logger().info('no. agents : %f'%msg.data[0])
        # self.get_logger().info('no. obstacles : %f'%msg.data[1])
        #t_max = 50
        #self.dynamic_visualization(t_max, self.D)
        self.num_agents      = int(msg.data[0])
        self.num_obstacles   = int(msg.data[1])
        self.positions       = msg.data[2:2*self.num_agents+2]
        self.goals           = msg.data[2*self.num_agents+2:]

        self.update_pedestrian_path()
        self.visualize()
    
    def update_pedestrian_path(self):
        if (len(self.pedestrian_pos) == 0):
            self.pedestrian_pos = np.zeros([self.num_agents, self.path_buffer*2], dtype='float')
            self.pedestrian_vel = np.zeros_like(self.pedestrian_pos)
            #print('path : ',self.pedestrian_pos)
        else:
            for i in range(self.num_agents):
                self.pedestrian_pos[i][:2*self.path_buffer-2] = self.pedestrian_pos[i][2:2*self.path_buffer]
                self.pedestrian_pos[i][2*self.path_buffer-2:] = self.positions[2*i:2*i+2]  
            #print('updated path : ', self.pedestrian_pos)
                
            for j in range(self.num_agents):
                self.pedestrian_vel[j][:2*self.path_buffer-2] = self.pedestrian_vel[j][2:2*self.path_buffer]
                self.pedestrian_vel[j][2*self.path_buffer-2:] = [(self.pedestrian_pos[j][-2] - self.pedestrian_pos[j][-4])/0.2,
                                                                 (self.pedestrian_pos[j][-1] - self.pedestrian_pos[j][-3])/0.2 ] 

        #print("vel :", self.pedestrian_vel)
    def pedestrian_state(self, timeStep , pd = 0):
        # Simulated pedestrian movement: moving in a sinusoidal pattern
        #pos = np.array([t, t + np.sin(t)])  # Position
        #vel = np.array([1, np.cos(t)])      # Velocity
        print("position: ",self.pedestrian_pos)
        print("velocity: ",self.pedestrian_vel)
        # print(self.pedestrian_pos[pd][2*self.path_buffer-2*timeStep-2:2*self.path_buffer-2*timeStep])
        # print(self.pedestrian_pos[pd][-2*timeStep-3:-2*timeStep])
        pos = tuple(self.pedestrian_pos[pd][2*self.path_buffer-2*timeStep-2:2*self.path_buffer-2*timeStep])
        vel = tuple(self.pedestrian_pos[pd][2*self.path_buffer-2*timeStep-2:2*self.path_buffer-2*timeStep])
        print("pos: ",pos)
        print("vel: ",vel)
        return pos, vel

    # Function to compute the angle between velocity and the vector to the destination
    # def compute_angle(self, pos, vel, dest):
    #     direction_to_dest = dest - pos
    #     try:
    #         angle = np.arccos(np.dot(vel, direction_to_dest) / (np.linalg.norm(vel) * np.linalg.norm(direction_to_dest)))
    #     except:
    #         print("Initializing")
    #         return 0
    #     return angle

    def compute_angle(self, pos, vel, dest):
        direction_to_dest = dest - pos
        norm_vel = np.linalg.norm(vel)
        norm_dir = np.linalg.norm(direction_to_dest)
        
        # Check if any of the vectors have zero magnitude
        if norm_vel == 0 or norm_dir == 0:
            print("Initializing")
            return 0
        
        # Clamp the value to avoid invalid input for arccos
        cos_theta = np.dot(vel, direction_to_dest) / (norm_vel * norm_dir)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)  # Ensure the value is between -1 and 1

        angle = np.arccos(cos_theta)
        return angle

    # Compute the probability P(sp(t+Δt)|Dm)
    def compute_probability(self, pos, vel, dest, sigma):
        angle = self.compute_angle(pos, vel, dest)
        prob = norm.pdf(angle, 0, sigma)  # Gaussian distribution
        return prob

    # Bayesian classifier for destination prediction
    def predict_destination(self, D, w=5):
        # Store recent pedestrian states
        recent_states = []
        
        for i in range(w):
            print("i is :",i)
            pos, vel = self.pedestrian_state(i)
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

    def visualize(self):
        pos, vel = self.pedestrian_state(timeStep= 0)
        
        plt.figure()
        plt.quiver(pos[0], pos[1], vel[0], vel[1], color='r', scale=10)  # Pedestrian velocity
        #plt.scatter(self.D[:, 0], self.D[:, 1], c='blue', label='Destinations')
        
        pred_dest = self.predict_destination(self.D)
        print("pred", pred_dest)
        #plt.scatter(pred_dest[0], pred_dest[1], c='green', label='Predicted Destination', marker='X')
        
        plt.xlim(0, 25)
        plt.ylim(0, 20)
        plt.legend()
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    goal_predictor = GoalPredictor()

    rclpy.spin(goal_predictor)
    goal_predictor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()