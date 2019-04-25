#!/usr/bin/env python
# coding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import argparse
import glob
import subprocess
import sys
import os
import yaml
import time

def get_YAML_data(filepath):
    data = None
    with open(filepath, 'r') as stream:
        try:
            data = yaml.safe_load(stream)
            print("Loaded YAML data from file", filepath)
        except yaml.YAMLError as exc:
            print(exc)
    return data

def get_color_vector(img, pos):
    # Inverted pos indexs because image is given as height x width
    return img[int(pos[1]), int(pos[0]), :]

def close_to_obstacles(pos, box_half_width, img):
    collides_with_obstacle = False
    img_height = img.shape[0]
    img_width = img.shape[1]
    
    min_x = int(max(0, pos[0] - box_half_width))
    max_x = int(min(img_width-1, pos[0] + box_half_width))
    min_y = int(max(0, pos[1] - box_half_width))
    max_y = int(min(img_height-1, pos[1] + box_half_width))

    # max to min iterations since probability of collision is higher
    # at the borders of the bounding box than at the center
    for i in range(max_x, min_x, -1):
        for j in range(max_y, min_y, -1):
            p = np.array([i, j])
            if np.allclose(np.linalg.norm(get_color_vector(img, p)), 0.0):
                collides_with_obstacle = True
                break

    return collides_with_obstacle

def discard_invalid_samples(img, samples, robot_pixel_radius, maxNumSamples):
    filtered_sample_indices = []
    for s_id in range(samples.shape[0]):
        pos = samples[s_id]
        if np.linalg.norm(get_color_vector(img, pos)) > 0.0:
            if not close_to_obstacles(pos, robot_pixel_radius, img):
                filtered_sample_indices.append(s_id)
                if len(filtered_sample_indices) >= maxNumSamples:
                    break

    return samples[filtered_sample_indices,:]

def plot_map(ax, map_file_path):
    img = plt.imread(map_file_path)
    img_height = img.shape[0]
    img_width = img.shape[1]
    ax.imshow(img)

def plot_samples(ax, samples, robot_radius):
    ax.scatter(samples[:, 0], samples[:,1], s=robot_radius/2.0)
    thetas = samples[:, 2]
    
    for i in range(thetas.shape[0]):
        ax.arrow(samples[i, 0], samples[i,1], 
                 robot_radius*np.cos(thetas[i]), robot_radius*np.sin(thetas[i]),
                 head_width=robot_radius/2.0, head_length=robot_radius/2.0, fc='k', ec='k')
    
def plot_problems(ax, problems, samples):
    for p in problems:
        start = samples[p[0]]
        goal = samples[p[1]]
        x = [start[0], goal[0]]
        y = [start[1], goal[1]]
        plt.plot(x, y)
    
def generate_samples(map_file_path, resolution, robot_radius, nSamples, hotspots=None):
    img = plt.imread(map_file_path)
    img_height = img.shape[0]
    img_width = img.shape[1]
    
    # Oversample to account for samples that will discarded due to obstacles
    sampleSize = nSamples * 4
    
    samples = np.zeros((sampleSize, 3))
    samples[:,0] = np.random.randint(0, img_width, samples.shape[0])
    samples[:,1] = np.random.randint(0, img_height, samples.shape[0])
    samples[:,2] = np.random.uniform(-np.pi, np.pi, samples.shape[0])
    samples = discard_invalid_samples(img, samples, robot_radius, nSamples)
    
    print("Discarded", nSamples - samples.shape[0], "samples as they were on or close to obstacles")    
    return samples

def generate_problem_scenarios(samples, nProblems):
    problems = np.zeros((nProblems, 2))
    
    used_samples = np.full((1, samples.shape[0]), False)
    
    for p_idx in range(nProblems):
        start = 0
        goal = 0
        
        # Find a unique start position
        while used_samples[0,start]:
            start = np.random.randint(0, samples.shape[0], 1)[0]
        used_samples[0, start] = True
            
        # Find a unique goal position
        while used_samples[0,goal]:
            goal = np.random.randint(0, samples.shape[0], 1)[0] 
        used_samples[0, goal] = True
        
        problems[p_idx, 0] = start
        problems[p_idx, 1] = goal

    return problems.astype(int)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_filename", help="Filename of the map image that should be used for dataset generation (ex. map1.png)")
    args = parser.parse_args()

    dir_name, _ = os.path.split(os.path.abspath(sys.argv[0]))
    map_file_path = os.path.abspath(dir_name + "/../../maps/" + args.map_filename)
    print(map_file_path)
    yaml_file_path = os.path.splitext(map_file_path)[0] + ".yaml"
    resolution = float(get_YAML_data(yaml_file_path)['resolution'])

    robot_radius = 10
    nSamples = 1000
    nProblems = 100

    samples = generate_samples(map_file_path, resolution, robot_radius, nSamples)
    print("Successfully generated", samples.shape[0], "samples")
    
    problems = generate_problem_scenarios(samples, nProblems)
    
    f = plt.figure(figsize=(20, 20))
    ax = f.subplots()
    plot_map(ax, map_file_path)
    plot_samples(ax, samples, robot_radius)
    plot_problems(ax, problems, samples)
    plt.savefig("/home/suvich15/Sushant/MAS/RnD_Resources/repositories/coordination_oru/maps/samples/map.svg",
                format='svg')



if __name__ == "__main__":
    main()
