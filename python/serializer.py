#!/usr/bin/env python
import yaml
import glob
import os
import numpy as np
import logging
from libarea import *
from xml.dom import minidom

class Serializer:
    def __init__(self):
        pass
        
    def read_config(self, filename, path=""):        
        try:            
            return yaml.load(open(os.path.join(path, filename), 'r'), yaml.CLoader)
        except:
            logging.error("Serializer: Can't read " + os.path.join(path, filename) + ". No such file")
            return None
        
    def read_process_covariance(self, path, filename=None):
        if filename == None:
            with open(path, 'r') as f:
                for line in f:
                    if "Process covariance:" in line:
                        return float(line.strip().split(": ")[1])
        
    def save_stats(self, stats, path=""):
        if not os.path.exists(path):
            os.makedirs(path)
        for file in glob.glob(os.path.join(path, "stats.yaml")):
            os.remove(file)
        with open(os.path.join(path, "stats.yaml"), 'w') as f:
            f.write(yaml.dump(stats, default_flow_style=False))
            
    '''def save_rewards(self, rewards, path=""):
        if not os.path.exists(path):
            os.makedirs(path)
        for file in glob.glob(os.path.join(path, "rewards.yaml")):
            os.remove(file)
        with open(os.path.join(path, "rewards.yaml"), 'w') as f:
            f.write(yaml.dump(rewards, default_flow_style=False))'''
            
    def save_list_data(self, data, path="", filename=""):
        if not os.path.exists(path):
            os.makedirs(path)        
        with open(os.path.join(path, filename), 'a') as f:            
            f.write(yaml.dump(data, default_flow_style=False))
            
    '''def save_avg_path_lengths(self, lengths, path=""):
        if not os.path.exists(path):
            os.makedirs(path)
        for file in glob.glob(os.path.join(path, "avg_path_lengths.yaml")):
            os.remove(file)
        with open(os.path.join(path, "avg_path_lengths.yaml"), 'w') as f:
            f.write(yaml.dump(lengths, default_flow_style=False))'''
            
    def load(self, filename, path=""):
        """
        General load function
        """
        try:
            with open(os.path.join(path, filename), 'r') as f:
                return yaml.load(f, yaml.CLoader)
        except:
            logging.error("Error opening " + str(filename) + ". No such file")
        return None
            
    def load_stats(self, filename, path=""):
        with open(os.path.join(path, filename), 'r') as f:
            return yaml.load(f, yaml.CLoader)
        
    def save_state_paths(self, paths, filename, path=""):
        if not os.path.exists(path):
            os.makedirs(path)        
        with open(os.path.join(path, filename), 'a') as f: 
            f.write(yaml.dump(paths, default_flow_style=False))
            
    def append_end_sign(self, filename, path=""):
        with open(os.path.join(path, filename), 'a') as f:
            f.write("###\n")           
                
            
    def save_paths(self, paths, filename, overwrite, path=""):
        if not os.path.exists(path):
            os.makedirs(path)
        path_arrays = []
        if overwrite:
            for file in glob.glob(os.path.join(path, filename)):
                os.remove(file)            
        else:
            try:
                path_arrays = self.load_paths(filename, path)
                for file in glob.glob(os.path.join(path, filename)):
                    os.remove(file)
            except:
                logging.warn("Serializer: Couldn't load paths.yaml")       
        for i in xrange(len(paths)):                       
            path_arr = []           
            for j in xrange(len(paths[i][0])):
                el = []
                el.extend(paths[i][0][j])
                el.extend(paths[i][1][j])
                el.extend(paths[i][2][j])
                path_arr.append(el)
            path_arrays.append(path_arr)  
        d = dict(paths = path_arrays)        
        with open(os.path.join(path, filename), 'a+') as f:
            f.write(yaml.dump(d, default_flow_style=False))
            
    def serialize_ik_solutions(self, ik_solutions, problem, path):
        for file in glob.glob(os.path.join("../problems/" + problem, 'goalstates.txt')):
            os.remove(file)
        with open(os.path.join(path + "/../problems/" + problem, 'goalstates.txt'), 'w') as f:
            for i in xrange(len(ik_solutions)):
                for j in xrange(len(ik_solutions[i])):
                    f.write(str(ik_solutions[i][j]) + " ")
                if not i == len(ik_solutions) - 1:
                    f.write("\n")
                    
    def deserialize_joint_angles(self, path="", file=""):
        float_arrs = []
        if not file == "":
            with open(os.path.join(path, file), 'r') as f:
                for line in f.readlines():
                    arr = line.split(" ")
                    
                    float_arr = [float(arr[k]) for k in xrange(0, len(arr) - 2)]
                    float_arrs.append(float_arr)
            return float_arrs
        return []
            
    def load_cartesian_coords(self, path=""):        
        with open(os.path.join(path, "cartesian_coords.yaml"), 'r') as f:
            return yaml.load(f, yaml.CLoader)  
    
    def load_paths(self, file, path=""):        
        paths = yaml.load(open(os.path.join(path, file), 'r'), yaml.CLoader)        
        return paths['paths']
    
    def load_obstacles(self, file=None, path=""):        
        if file == None:
            obstacles = []            
            for file in glob.glob(os.path.join(path, "obst*")):                
                if not "map" in file and not "~" in file:
                    obstacles.append(yaml.load(open(file, 'r'), yaml.CLoader)[0])            
            return obstacles
        return yaml.load(open(os.path.join(path, file), 'r'), yaml.CLoader)
    
    