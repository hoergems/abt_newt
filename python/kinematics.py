import numpy as np

class Kinematics:
    def __init__(self, num_links, dim):
        #theta = [1.5, 2.7]
        self.num_links = num_links
        self.femour_transform = 0.0
        if dim == 3:
            self.femour_transform = -np.pi / 2      
        
    def get_link_n_position(self, state, n):
        t1 = self.dh_transformation(state[0], 0.0, 1.0, self.femour_transform)
        t2 = self.dh_transformation(state[1], 0.0, 1.0, 0.0)
        t3 = self.dh_transformation(state[2], 0.0, 1.0, 0.0)
        if n == 0:
            return np.dot(t1, np.array([0.0, 0.0, 0.0, 1.0]))
        elif n == 1:
            return np.dot(np.dot(t1, t2), np.array([0.0, 0.0, 0.0, 1.0])) 
        elif n == 2:
            return np.dot(np.dot(np.dot(t1, t2), t3), np.array([0.0, 0.0, 0.0, 1.0]))
        
    def get_end_effector_position(self, state):
        """
        Gets the end effector position from a joint state
        """  
        return self.get_link_n_position(state, 2)
        
    def dh_transformation(self, sigma_n, d_n, a_n, alpha_n):
        """
        Describes a DH-transformation with the given DH parameters
        """
        return np.array([[np.cos(sigma_n), -1.0 * np.sin(sigma_n) * np.cos(alpha_n), np.sin(sigma_n) * np.sin(alpha_n), a_n * np.cos(sigma_n)],
                         [np.sin(sigma_n), np.cos(sigma_n) * np.cos(alpha_n), -1.0 * np.cos(sigma_n) * np.sin(alpha_n), a_n * np.sin(sigma_n)],
                         [0.0, np.sin(alpha_n), np.cos(alpha_n), d_n],
                         [0.0, 0.0, 0.0, 1.0]])
    
    
        
if __name__ == "__main__":
    Kinematics()