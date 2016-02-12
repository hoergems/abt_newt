import sys
import numpy as np
import plot as Plot
import glob
import os
import logging
import scipy
from kin import *
from util import *
from serializer import Serializer
from environment import *
from EMD import *

class PlotStats:
    def __init__(self, save):
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
        if not os.path.isdir("stats"):
            os.makedirs("stats")      
        self.save = save
        self.utils = Utils()
        logging.info("Loading cartesian coordinates...") 
        serializer = Serializer()
        
        rave_env = Environment()
        f = "stats/model/test.urdf"
        rave_env.initializeEnvironment(f)
        self.link_dimensions = rave_env.getRobotLinkDimensions()
        rave_env.destroy()
        
        self.setup_kinematics(serializer, "stats")
        self.process_covariances = []
        for logfile in glob.glob("stats" + "/*.log"):
            self.process_covariances.append(serializer.read_process_covariance(logfile))
        #arr = serializer.deserialize_joint_angles(path="stats", file="state_path1.txt")        
        #self.plot_state_path(serializer, arr)
        #sleep()
        
        self.create_video(serializer, "stats")
        logging.info("plotting paths")  
        try:  
            self.plot_paths(serializer)
        except:
            logging.warn("Paths could not be plotted")        
        logging.info("Reading config") 
        config = serializer.read_config("config.yaml", path="stats")
            
        #try:
        #self.plot_end_effector_paths(serializer, plot_scenery=True, plot_manipulator=True)
        #except:
        #    print "Error. End effector paths could not be plotted"
        
        logging.info("Plotting mean planning times")
        self.plot_mean_planning_times(serializer, 
                                      dir="stats", 
                                      filename="mean_planning_times_per_step*.yaml", 
                                      output="mean_planning_times_per_step.pdf")
        self.plot_mean_planning_times(serializer, 
                                      dir="stats", 
                                      filename="mean_planning_times_per_run*.yaml", 
                                      output="mean_planning_times_per_run.pdf")
        
        self.plot_particles(serializer, particle_limit=config['particle_plot_limit'])
        
        try:
            self.plot_paths(serializer, best_paths=True)
        except:
            logging.warn("Best_paths could not be plotted")
            
        try:
            cart_coords = serializer.load_cartesian_coords(path="stats")
        except:
            logging.warn("Cartesian_coords could not be plotted")
        
        logging.info("Plotting average distance to goal" )       
        try:
            self.plot_average_dist_to_goal(serializer, cart_coords)
        except:
            logging.warn("Average distance to goal could not be plotted")
        
        logging.info("plotting mean rewards")
        try:
            self.plot_mean_rewards(serializer)
        except Exception as e:
            logging.warn("Mean rewards could not be plotted: " + str(e))       
        
        logging.info("Plotting EMD graph...")       
        try:
            self.plot_emd_graph(serializer, cart_coords) 
        except:
            logging.warn("EMD graphs could not be plotted")
        
        logging.info("Plotting histograms...") 
        try:            
            self.save_histogram_plots(serializer, cart_coords) 
        except:
            logging.warn("Histograms could not be plotted")  
        
    def clear_stats(self):
        for file in glob.glob("stats/*"):
            os.remove(file)
            
    def setup_kinematics(self, serializer, dir='stats'):
        config = serializer.read_config("config.yaml", path=dir)        
        model_file = os.getcwd() + "/" + dir + "/model/model.xml"
        if config['workspace_dimensions'] == 3:
            model_file = os.getcwd() + "/" + dir + "/model/model3D.xml"
        print "model file: " + model_file
        axis = v2_int()
        ax1 = v_int()
        ax2 = v_int()
        ax1[:] = [0, 0, 1]
        if config['workspace_dimensions'] == 2:
            ax2[:] = [0, 0, 1]            
        elif config['workspace_dimensions'] == 3:
            ax2[:] = [0, 1, 0]
        axis[:] = [ax1, ax2, ax1]
        self.kinematics = Kinematics()
        self.kinematics.setLinksAndAxis(self.link_dimensions, axis)
            
    def plot_reward_variances(self, serializer):
        stats = serializer.load_stats('stats.yaml', path="stats")
        m_cov = stats['m_cov']
        
            
    def plot_mean_rewards(self, serializer):        
        stats = serializer.load_stats('stats.yaml', path="stats")
        m_cov = stats['m_cov']
        rewards = serializer.load_stats('rewards.yaml', path="stats")
        mean_rewards_data = []
        variance_data = []
        min_mean_reward = float("inf")
        max_mean_reward = -float("inf")
        min_variance = float("inf")
        max_variance = -float("inf")
        for k in xrange(len(m_cov)):            
            n, min_max, mean, var, skew, kurt = scipy.stats.describe(np.array(rewards[k]))       
            mean_rewards_data.append(np.array([m_cov[k], mean]))
            variance_data.append(np.array([m_cov[k], var]))
            if min_max[0] < min_mean_reward:
                min_mean_reward = min_max[0]
            if min_max[1] > max_mean_reward:
                max_mean_reward = min_max[1]            
            if var > max_variance:
                max_variance = var
            if var < min_variance:
                min_variance = var
        Plot.plot_2d_n_sets([np.array(mean_rewards_data)],
                            xlabel="joint covariance",
                            ylabel="mean reward",
                            x_range=[m_cov[0], m_cov[-1]],
                            y_range=[min_mean_reward - 0.1, max_mean_reward + 0.1],
                            show_legend=False,
                            save=self.save,
                            filename="stats/mean_rewards.png")
        Plot.plot_2d_n_sets([np.array(variance_data)],
                            xlabel="joint covariance",
                            ylabel="reward variance",
                            x_range=[m_cov[0], m_cov[-1]],
                            y_range=[min_variance - 0.1, max_variance + 0.1],
                            show_legend=False,
                            save=self.save,
                            filename="stats/reward_variance.png")
        
    def plot_mean_planning_times(self, serializer, dir="stats", filename="", output=""): 
        if filename == "":
            filename = "mean_planning_times_per_step*.yaml"
        if output == "":
            output = "mean_planning_times_per_step.pdf"       
        stats = serializer.load_stats('stats.yaml', path=dir)
        m_cov = stats['m_cov']
        sets = []
        labels = []
        mean_planning_times = []
        for file in glob.glob(os.path.join(os.path.join(dir, filename))):
            file_str = file
            try:
                
                file_str = file.split("/")[1].split(".")[0].split("_")[5] + "_" + file.split("/")[1].split(".")[0].split("_")[6]
            except:
                pass
                   
            #mean_rewards = serializer.load_stats('rewards.yaml', path="stats")
            mean_planning_times.append(serializer.load_stats(file))            
            data = []
            for k in xrange(len(m_cov)):
                data.append(np.array([m_cov[k], mean_planning_times[-1][k]]))
            sets.append(np.array(data))            
            labels.append(file_str)        
        if not len(mean_planning_times) == 0:
            min_m = [min(m) for m in mean_planning_times]
            max_m = [max(m) for m in mean_planning_times]
            Plot.plot_2d_n_sets(sets,
                                labels=labels,
                                xlabel="joint covariance",
                                ylabel="mean planning times (seconds)",
                                x_range=[m_cov[0], m_cov[-1]],
                                y_range=[min(min_m), max(max_m) * 1.05],
                                show_legend=True,
                                save=self.save,
                                filename=dir + "/" + output)         
        
    def plot_average_dist_to_goal(self, serializer, cart_coords):
        config = serializer.read_config("config.yaml", path="stats")
        stats = serializer.load_stats('stats.yaml', path="stats")
        m_cov = stats['m_cov']
        data = []
        max_avg_distance = 0.0
        for k in xrange(len(m_cov)):            
            dists = []            
            for coords in cart_coords[k]:
                dists.append(np.linalg.norm(np.array(coords) - np.array(config['goal_position'])))            
            avg_distance = 0.0
            for d in dists:
                avg_distance += d
            avg_distance /= len(dists)
            if avg_distance > max_avg_distance:
                max_avg_distance = avg_distance                       
            data.append(np.array([m_cov[k], avg_distance]))
        
        Plot.plot_2d_n_sets([np.array(data)],
                            xlabel="joint covariance",
                            ylabel="average distance to goal",
                            x_range=[m_cov[0], m_cov[-1]],
                            y_range=[0, max_avg_distance],
                            show_legend=False,
                            save=self.save,
                            filename="stats/avg_distance.png")
        
    def plot_emd_graph(self, serializer, cartesian_coords):
        stats = serializer.load_stats('stats.yaml', path="stats") 
        config = serializer.read_config('config.yaml', path="stats")       
        #emd = stats['emd']
        m_cov = stats['m_cov']
        
        emds = []
        for k in xrange(len(cartesian_coords)):
            #cart_coords.append([cartesian_coords[i] for i in xrange(len(cartesian_coords))])                
            emds.append(calc_EMD(cartesian_coords[k], config['num_bins']))
        
        arr = np.array([np.array([m_cov[i], emds[i]]) for i in xrange(len(emds))])
        Plot.plot_2d_n_sets([arr], 
                            xlabel='joint covariance', 
                            ylabel='Wasserstein distance', 
                            x_range=[m_cov[0], m_cov[-1]], 
                            y_range=[0, max(emds)],
                            show_legend=False,
                            save=self.save,
                            path="stats",
                            filename="emd.png")        
        
    def save_histogram_plots(self, serializer, cart_coords):
        config = serializer.read_config("config.yaml", path="stats")
        for file in glob.glob(os.path.join("stats", "hist*")):
            os.remove(file)
        for k in xrange(len(cart_coords)):                    
            X = np.array([cart_coords[k][i][0] for i in xrange(len(cart_coords[0]))])
            Y = np.array([cart_coords[k][i][1] for i in xrange(len(cart_coords[0]))])
            histogram_range = [[-3.1, 3.1], [-3.1, 3.1]]
            H, xedges, yedges = get_2d_histogram(X, Y, histogram_range, bins=config['num_bins'])        
            Plot.plot_histogram(H, xedges, yedges, save=self.save, path="stats", filename="hist"+ str(k) + ".png")
            
    def plot_state_path(self, serializer, state_path):
        config = serializer.read_config('config.yaml', path="stats")
        print "current dir " + os.getcwd()
        kinematics = Kinematics()
        kinematics.setLinksAndAxis(self.link_dimensions, axis)
        kinematics = Kinematics(config['num_links'], config['workspace_dimensions'])
        obstacles = serializer.load_obstacles(path="stats/obstacles")
        environment = serializer.load_environment() 
        for file in glob.glob(os.path.join("stats", "state_paths*.png")):
            os.remove(file)  
        sets = []
        color_map = []
        for obstacle in environment:
            point1 = [obstacle[0][0] - obstacle[1][0] / 2.0, obstacle[0][1] - obstacle[1][1] / 2.0, obstacle[0][2] - obstacle[1][2] / 2.0]
            point2 = [obstacle[0][0] - obstacle[1][0] / 2.0, obstacle[0][1] - obstacle[1][1] / 2.0, obstacle[0][2] + obstacle[1][2] / 2.0]
            point3 = [obstacle[0][0] - obstacle[1][0] / 2.0, obstacle[0][1] + obstacle[1][1] / 2.0, obstacle[0][2] - obstacle[1][2] / 2.0]
            point4 = [obstacle[0][0] - obstacle[1][0] / 2.0, obstacle[0][1] + obstacle[1][1] / 2.0, obstacle[0][2] + obstacle[1][2] / 2.0]
            point5 = [obstacle[0][0] + obstacle[1][0] / 2.0, obstacle[0][1] - obstacle[1][1] / 2.0, obstacle[0][2] - obstacle[1][2] / 2.0]
            point6 = [obstacle[0][0] + obstacle[1][0] / 2.0, obstacle[0][1] - obstacle[1][1] / 2.0, obstacle[0][2] + obstacle[1][2] / 2.0]
            point7 = [obstacle[0][0] + obstacle[1][0] / 2.0, obstacle[0][1] + obstacle[1][1] / 2.0, obstacle[0][2] - obstacle[1][2] / 2.0]
            point8 = [obstacle[0][0] + obstacle[1][0] / 2.0, obstacle[0][1] + obstacle[1][1] / 2.0, obstacle[0][2] + obstacle[1][2] / 2.0]
            if config['workspace_dimensions'] == 2:
                sets.append(np.array([point1, point3]))
                sets.append(np.array([point3, point7]))
                sets.append(np.array([point7, point5]))
                sets.append(np.array([point5, point1]))
                color_map.extend(['#000000' for t in xrange(4)])
            elif config['workspace_dimensions'] == 3:
                sets.append(np.array([point1, point3]))
                sets.append(np.array([point3, point7]))
                sets.append(np.array([point7, point5]))
                sets.append(np.array([point5, point1]))
                                
                sets.append(np.array([point2, point4]))
                sets.append(np.array([point4, point8]))
                sets.append(np.array([point8, point6]))
                sets.append(np.array([point6, point2]))
                                
                sets.append(np.array([point1, point2]))
                sets.append(np.array([point3, point4]))
                sets.append(np.array([point7, point8]))
                sets.append(np.array([point5, point6]))
                color_map.extend(['#000000' for t in xrange(12)])
        balls = [[config['goal_position'], config["goal_radius"]]]
        color_map.extend(['green' for i in xrange(len(balls))])    
        for i in xrange(len(state_path)): 
            temp_sets = []
            color_map_temp = []                                              
            link_1_position = kinematics.get_link_n_position(state_path[i], 0)
            link_2_position = kinematics.get_link_n_position(state_path[i], 1)
            link_3_position = kinematics.get_link_n_position(state_path[i], 2)
                        
            temp_sets.append(np.array([[0.0, 0.0], [link_1_position[0], link_1_position[1]]]))
            temp_sets.append(np.array([[link_1_position[0], link_1_position[1]], 
                                       [link_2_position[0], link_2_position[1]]]))
            temp_sets.append(np.array([[link_2_position[0], link_2_position[1]], 
                                       [link_3_position[0], link_3_position[1]]]))
            color_map_temp.extend(['#0000ff' for t in xrange(3)])
            temp_sets.extend(sets)
            color_map_temp.extend(color_map) 
            
            if config['workspace_dimensions'] == 2:
                circles = []
                for ball in balls:
                    circles.append([ball[0][0], ball[0][1], ball[1]])
                Plot.plot_2d_n_sets(temp_sets,
                                    circles=circles,
                                    xlabel="x",
                                    ylabel="y",
                                    x_range=[-3.5, 3.5], 
                                    y_range=[-3.5, 3.5],
                                    plot_type="lines",
                                    show_legend=False,
                                    color_map=color_map_temp,
                                    save=self.save,
                                    path="stats",
                                    filename="state_path" + str(i) + ".png")
                
    def create_video(self, serializer, dir='stats'):
        try:
            os.makedirs(dir + "/mov")
        except Exception as e:
            print e
        config = serializer.read_config("config.yaml", path=dir)
        utils = Utils()        
        
        map_size = sum([link_dimension[0] for link_dimension in self.link_dimensions])
        environment = serializer.load_environment(path=dir + "/environment")        
        sets = []
        color_map = []
        plot_particles = True
        for obstacle in environment:
            point1 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] - obstacle[1][1], obstacle[0][2] - obstacle[1][2]]
            point2 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] - obstacle[1][1], obstacle[0][2] + obstacle[1][2]]
            point3 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] + obstacle[1][1], obstacle[0][2] - obstacle[1][2]]
            point4 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] + obstacle[1][1], obstacle[0][2] + obstacle[1][2]]
            point5 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] - obstacle[1][1], obstacle[0][2] - obstacle[1][2]]
            point6 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] - obstacle[1][1], obstacle[0][2] + obstacle[1][2]]
            point7 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] + obstacle[1][1], obstacle[0][2] - obstacle[1][2]]
            point8 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] + obstacle[1][1], obstacle[0][2] + obstacle[1][2]]
            if config['workspace_dimensions'] == 2:
                sets.append(np.array([point1, point3]))
                sets.append(np.array([point3, point7]))
                sets.append(np.array([point7, point5]))
                sets.append(np.array([point5, point1]))
                color_map.extend(['#000000' for t in xrange(4)])
            elif config['workspace_dimensions'] == 3:
                sets.append(np.array([point1, point3]))
                sets.append(np.array([point3, point7]))
                sets.append(np.array([point7, point5]))
                sets.append(np.array([point5, point1]))
                                
                sets.append(np.array([point2, point4]))
                sets.append(np.array([point4, point8]))
                sets.append(np.array([point8, point6]))
                sets.append(np.array([point6, point2]))
                                
                sets.append(np.array([point1, point2]))
                sets.append(np.array([point3, point4]))
                sets.append(np.array([point7, point8]))
                sets.append(np.array([point5, point6]))
                color_map.extend(['#000000' for t in xrange(12)])
        balls = [[config['goal_position'], config["goal_radius"]]]
        color_map.extend(['green' for i in xrange(len(balls))])
        state = None
        particles_begin = False
        s_first = False
        particles = []
        cov_num  = -1
        for process_covariance in self.process_covariances:
            for file in glob.glob(dir + "/*.log"):                
                if str(process_covariance) in file:                    
                    cov_num += 1                   
                    run_num = -1
                    step_num = -1
                    with open(file, 'r') as f:                        
                        for line in f:                                                       
                            if "RUN #" in line or "Run #" in line:                                
                                run_num += 1
                                step_num = -1
                            elif "S_ESTIMATED:" in line:
                                line_arr = line.split(":")[1].strip().split(" ")
                                estimated_state = [float(line_arr[j]) for j in xrange(len(line_arr))]                           
                            elif "PARTICLES BEGIN" in line:
                                particles_begin = True
                                particles = []
                            elif "p:" in line and particles_begin and not "step" in line:
                                if len(particles) < config['particle_plot_limit']:
                                    particle = line.split(":")[1].strip().split(" ")
                                    particle = [float(p) for p in particle]
                                    particles.append(particle)
                            elif "PARTICLES END" in line:
                                particles_begin = False
                                
                            elif "S:" in line:
                                line_arr = line.split(":")[1].strip().split(" ")
                                state = [float(line_arr[j]) for j in xrange(len(line_arr) / 2)]
                                if not len(particles) == 0:
                                    temp_sets = []
                                    color_map_temp = []
                                    step_num += 1  
                                    if plot_particles:                                                                     
                                        for particle in particles:
                                            angles = v_double()
                                            angles[:] = particle
                                            link_1_position = self.kinematics.getPositionOfLinkN(angles, 1)
                                            link_2_position = self.kinematics.getPositionOfLinkN(angles, 2)
                                            link_3_position = self.kinematics.getPositionOfLinkN(angles, 3)
                                            temp_sets.append(np.array([[0.0, 0.0, 0.0], [link_1_position[0], link_1_position[1], link_1_position[2]]]))
                                            temp_sets.append(np.array([[link_1_position[0], link_1_position[1], link_1_position[2]], 
                                                                       [link_2_position[0], link_2_position[1], link_2_position[2]]]))
                                            temp_sets.append(np.array([[link_2_position[0], link_2_position[1], link_2_position[2]], 
                                                                       [link_3_position[0], link_3_position[1], link_3_position[2]]]))
                                            color_map_temp.extend(['#aaabbb' for t in xrange(3)])                                                              
                                    angles = v_double()
                                    angles[:] = state   
                                    img_filename = "img_" + str(cov_num) + "_" + str(run_num) + "_" + "0000"
                                    if step_num < 10:
                                        img_filename += "0"
                                    img_filename += str(step_num) + ".png"                                 
                                    #img_filename = "img_" + str(cov_num) + "_" + str(run_num) + "_" + "0000" + str(step_num) + ".png"
                                    link_1_position = self.kinematics.getPositionOfLinkN(angles, 1)
                                    link_2_position = self.kinematics.getPositionOfLinkN(angles, 2)
                                    link_3_position = self.kinematics.getPositionOfLinkN(angles, 3)
                                    temp_sets.append(np.array([[0.0, 0.0, 0.0], [link_1_position[0], link_1_position[1], link_1_position[2]]]))
                                    temp_sets.append(np.array([[link_1_position[0], link_1_position[1], link_1_position[2]], 
                                                              [link_2_position[0], link_2_position[1], link_2_position[2]]]))
                                    temp_sets.append(np.array([[link_2_position[0], link_2_position[1], link_2_position[2]], 
                                                              [link_3_position[0], link_3_position[1], link_3_position[2]]]))
                                    color_map_temp.extend(['#0000ff' for n in xrange(3)])                                
                                    
                                    '''angles = v_double()
                                    
                                    angles[:] = estimated_state
                                    link_1_position = self.kinematics.getPositionOfLinkN(angles, 1)
                                    link_2_position = self.kinematics.getPositionOfLinkN(angles, 2)
                                    link_3_position = self.kinematics.getPositionOfLinkN(angles, 3)
                                    temp_sets.append(np.array([[0.0, 0.0, 0.0], [link_1_position[0], link_1_position[1], link_1_position[2]]]))
                                    temp_sets.append(np.array([[link_1_position[0], link_1_position[1], link_1_position[2]], 
                                                              [link_2_position[0], link_2_position[1], link_2_position[2]]]))
                                    temp_sets.append(np.array([[link_2_position[0], link_2_position[1], link_2_position[2]], 
                                                              [link_3_position[0], link_3_position[1], link_3_position[2]]]))
                                    color_map_temp.extend(['#ff0000' for n in xrange(3)])'''
                                    temp_sets.extend(sets)
                                    color_map_temp.extend(color_map)        
                                                              
                                    if config['workspace_dimensions'] == 2:
                                        circles = []
                                        for ball in balls:
                                            circles.append([ball[0][0], ball[0][1], ball[1]])
                                        Plot.plot_2d_n_sets(temp_sets,
                                                            circles=circles,
                                                            xlabel="x",
                                                            ylabel="y",
                                                            x_range=[-map_size * 1.1, map_size * 1.1], 
                                                            y_range=[-map_size * 1.1, map_size * 1.1],
                                                            plot_type="lines",
                                                            show_legend=False,
                                                            color_map=color_map_temp,
                                                            save=self.save,
                                                            path=dir + "/mov",
                                                            filename=img_filename)
        sleep
        
            
    def plot_particles(self, serializer, particle_limit=0):
        config = serializer.read_config('config.yaml', path="stats")
        for file in glob.glob(os.path.join("stats", "particles*.png")):
            os.remove(file)
        if config['plot_particles']:
            particles = serializer.load("particles.yaml", path="stats")
        state_paths = serializer.load("state_paths.yaml", path="stats")        
        
        map_size = sum([link_dimension[0] for link_dimension in self.link_dimensions])
        axis = v2_int()
        ax1 = v_int()
        ax2 = v_int()
        ax1[:] = [0, 0, 1]
        if config['workspace_dimensions'] == 2:
            ax2[:] = [0, 0, 1]            
        elif config['workspace_dimensions'] == 3:
            ax2[:] = [0, 1, 0]
            
        axis[:] = [ax1, ax2, ax1] 
        kinematics = Kinematics()
        kinematics.setLinksAndAxis(self.link_dimensions, axis)
        obstacles = serializer.load_obstacles(path="stats/obstacles")
        environment = serializer.load_environment()        
            
        for file in glob.glob(os.path.join("stats", "particles*.png")):
            os.remove(file)
                
        sets = [] 
        color_map = []
        for obstacle in environment:
            point1 = [obstacle[0][0] - obstacle[1][0] / 2.0, obstacle[0][1] - obstacle[1][1] / 2.0, obstacle[0][2] - obstacle[1][2] / 2.0]
            point2 = [obstacle[0][0] - obstacle[1][0] / 2.0, obstacle[0][1] - obstacle[1][1] / 2.0, obstacle[0][2] + obstacle[1][2] / 2.0]
            point3 = [obstacle[0][0] - obstacle[1][0] / 2.0, obstacle[0][1] + obstacle[1][1] / 2.0, obstacle[0][2] - obstacle[1][2] / 2.0]
            point4 = [obstacle[0][0] - obstacle[1][0] / 2.0, obstacle[0][1] + obstacle[1][1] / 2.0, obstacle[0][2] + obstacle[1][2] / 2.0]
            point5 = [obstacle[0][0] + obstacle[1][0] / 2.0, obstacle[0][1] - obstacle[1][1] / 2.0, obstacle[0][2] - obstacle[1][2] / 2.0]
            point6 = [obstacle[0][0] + obstacle[1][0] / 2.0, obstacle[0][1] - obstacle[1][1] / 2.0, obstacle[0][2] + obstacle[1][2] / 2.0]
            point7 = [obstacle[0][0] + obstacle[1][0] / 2.0, obstacle[0][1] + obstacle[1][1] / 2.0, obstacle[0][2] - obstacle[1][2] / 2.0]
            point8 = [obstacle[0][0] + obstacle[1][0] / 2.0, obstacle[0][1] + obstacle[1][1] / 2.0, obstacle[0][2] + obstacle[1][2] / 2.0]
            if config['workspace_dimensions'] == 2:
                sets.append(np.array([point1, point3]))
                sets.append(np.array([point3, point7]))
                sets.append(np.array([point7, point5]))
                sets.append(np.array([point5, point1]))
                color_map.extend(['#000000' for t in xrange(4)])
            elif config['workspace_dimensions'] == 3:
                sets.append(np.array([point1, point3]))
                sets.append(np.array([point3, point7]))
                sets.append(np.array([point7, point5]))
                sets.append(np.array([point5, point1]))
                                
                sets.append(np.array([point2, point4]))
                sets.append(np.array([point4, point8]))
                sets.append(np.array([point8, point6]))
                sets.append(np.array([point6, point2]))
                                
                sets.append(np.array([point1, point2]))
                sets.append(np.array([point3, point4]))
                sets.append(np.array([point7, point8]))
                sets.append(np.array([point5, point6]))
                color_map.extend(['#000000' for t in xrange(12)])
        balls = [[config['goal_position'], config["goal_radius"]]]
        color_map.extend(['green' for i in xrange(len(balls))])       
        for i in xrange(len(state_paths)):
            for j in xrange(len(state_paths[i])):                
                for k in xrange(len(state_paths[i][j])):
                    temp_sets = []
                    color_map_temp = []
                    if config['plot_particles'] and not k == 0:
                        for l in xrange(len(particles[i][j][k - 1])):
                            if not particle_limit == 0:
                                if l > particle_limit + 1:
                                    continue
                            angles = v_double();
                            angles[:] = particles[i][j][k - 1][l]
                            link_1_position = kinematics.getPositionOfLinkN(angles, 1)
                            link_2_position = kinematics.getPositionOfLinkN(angles, 2)
                            link_3_position = kinematics.getPositionOfLinkN(angles, 3)
                            temp_sets.append(np.array([[0.0, 0.0, 0.0], [link_1_position[0], link_1_position[1], link_1_position[2]]]))
                            temp_sets.append(np.array([[link_1_position[0], link_1_position[1], link_1_position[2]], 
                                                      [link_2_position[0], link_2_position[1], link_2_position[2]]]))
                            temp_sets.append(np.array([[link_2_position[0], link_2_position[1], link_2_position[2]], 
                                                      [link_3_position[0], link_3_position[1], link_3_position[2]]]))
                            color_map_temp.extend(['#aaabbb' for t in xrange(3)])                                                        
                        angles = v_double();
                        angles[:] = state_paths[i][j][k]   
                        link_1_position = kinematics.getPositionOfLinkN(angles, 1)
                        link_2_position = kinematics.getPositionOfLinkN(angles, 2)
                        link_3_position = kinematics.getPositionOfLinkN(angles, 3)
                            
                        temp_sets.append(np.array([[0.0, 0.0, 0.0], [link_1_position[0], link_1_position[1], link_1_position[2]]]))
                        temp_sets.append(np.array([[link_1_position[0], link_1_position[1], link_1_position[2]], 
                                                  [link_2_position[0], link_2_position[1], link_2_position[2]]]))
                        temp_sets.append(np.array([[link_2_position[0], link_2_position[1], link_2_position[2]], 
                                                  [link_3_position[0], link_3_position[1], link_3_position[2]]]))
                        color_map_temp.extend(['#0000ff' for t in xrange(3)])
                        
                        """
                        Plot the goal area
                        """                       
                        
                        temp_sets.extend(sets)
                        color_map_temp.extend(color_map)                       
                        if config['workspace_dimensions'] == 2:
                            circles = []
                            for ball in balls:
                                circles.append([ball[0][0], ball[0][1], ball[1]])
                            Plot.plot_2d_n_sets(temp_sets,
                                                circles=circles,
                                                xlabel="x",
                                                ylabel="y",
                                                x_range=[-map_size * 1.1, map_size * 1.1], 
                                                y_range=[-map_size * 1.1, map_size * 1.1],
                                                plot_type="lines",
                                                show_legend=False,
                                                color_map=color_map_temp,
                                                save=self.save,
                                                path="stats",
                                                filename="particles" + str(i) + "_" + str(j) + "_" + str(k) + ".png")
                        elif config['workspace_dimensions'] == 3:
                             Plot.plot_3d_n_sets(sets=temp_sets,
                                                balls=balls, 
                                                colormap=color_map_temp, 
                                                show_legend=False, 
                                                save=self.save, 
                                                path="stats", 
                                                filename="particles" + str(i) + "_" + str(j) + "_" + str(k) + ".png")
            
            
    def plot_end_effector_paths(self, serializer, plot_scenery=False, plot_manipulator=False): 
        config = serializer.read_config('config.yaml', path="stats")       
        ee_paths = serializer.load("ee_paths.yaml", path="stats")
        state_paths = serializer.load("state_paths.yaml", path="stats")
        kinematics = Kinematics(config['num_links'])
        for file in glob.glob(os.path.join("stats", "ee_paths*.png")):
            os.remove(file)      
        for i in xrange(len(ee_paths)):
            sets = [np.array(ee_path) for ee_path in ee_paths[i]]                
            if plot_scenery:
                obstacles = serializer.load_obstacles(path="stats/obstacles")
                if not obstacles == None:
                    for obstacle in obstacles:
                        point1 = [obstacle[0] - obstacle[2] / 2.0, obstacle[1] - obstacle[3] / 2.0]
                        point2 = [obstacle[0] - obstacle[2] / 2.0, obstacle[1] + obstacle[3] / 2.0]
                        point3 = [obstacle[0] + obstacle[2] / 2.0, obstacle[1] + obstacle[3] / 2.0]
                        point4 = [obstacle[0] + obstacle[2] / 2.0, obstacle[1] - obstacle[3] / 2.0]
                        sets.append(np.array([point1, point2]))
                        sets.append(np.array([point2, point3]))
                        sets.append(np.array([point3, point4]))
                        sets.append(np.array([point4, point1]))
            if plot_manipulator:
                for j in xrange(len(state_paths[i])):
                    for k in xrange(len(state_paths[i][j])):                        
                        link_1_position = kinematics.get_link_n_position(state_paths[i][j][k], 1)
                        link_2_position = kinematics.get_link_n_position(state_paths[i][j][k], 2)
                        link_3_position = kinematics.get_link_n_position(state_paths[i][j][k], 3)
                        
                        sets.append(np.array([[0.0, 0.0], [link_1_position[0], link_1_position[1]]]))
                        sets.append(np.array([[link_1_position[0], link_1_position[1]], 
                                              [link_2_position[0], link_2_position[1]]]))
                        sets.append(np.array([[link_2_position[0], link_2_position[1]], 
                                              [link_3_position[0], link_3_position[1]]]))
            Plot.plot_2d_n_sets(sets,
                                xlabel="x",
                                ylabel="y",
                                x_range=[-3.5, 3.5], 
                                y_range=[-3.5, 3.5],
                                plot_type="lines",
                                show_legend=False,
                                save=self.save,
                                path="stats",
                                filename="ee_paths" + str(i) + ".png")
            
    
            
    def plot_paths(self, serializer, best_paths=False):
        config = serializer.read_config('config.yaml', path="stats")
        dim = config['num_links']
        kinematics = Kinematics(dim)
        if best_paths:
            paths = serializer.load_paths("best_paths.yaml", path="stats")
            filename = "best_paths.png"
        else:
            paths = serializer.load_paths("paths.yaml", path="stats")            
            filename = "paths.png"
        sets = []
        for path in paths:
            path_coords = []
            for elem in path:
                state = [elem[i] for i in xrange(dim)]
                path_coords.append(kinematics.get_end_effector_position(state))
            sets.append(np.array(path_coords))               
        Plot.plot_2d_n_sets(sets, 
                            xlabel='x', 
                            ylabel='y', 
                            x_range=[-3.5, 3.5], 
                            y_range=[-3.5, 3.5],
                            plot_type="lines",
                            show_legend=False,
                            save=self.save,
                            path="stats",
                            filename=filename)    
    
        
if __name__ == "__main__":
    if len(sys.argv) > 1:
        if "save" in sys.argv[1]:
            PlotStats(True)
            sys.exit()       
        PlotStats(False)
        sys.exit()   
    PlotStats(False)
