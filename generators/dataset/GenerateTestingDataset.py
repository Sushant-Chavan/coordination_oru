#!/usr/bin/env python
# coding: utf-8

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import argparse
import glob
import subprocess
import sys
import os
import yaml
import time
import pandas as pd

class DatasetGenerator():
    def __init__(self, args):
        self.root_dir = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../")
        self.map_filename = args.map_filename
        self.robot_radius = int(args.robot_radius)
        self.oversamplingFactor = args.oversampling
        self.save_dbg_image = args.dbg_image
        self.nRobotsList = args.nRobots

        self.map_file_path = os.path.abspath(self.root_dir + "/maps/" + self.map_filename)
        self.map_name = os.path.splitext(self.map_filename)[0]

        self.img = plt.imread(self.map_file_path)
        self.img_height = self.img.shape[0]
        self.img_width = self.img.shape[1]

        yaml_file_path = os.path.splitext(self.map_file_path)[0] + ".yaml"
        self.resolution = float(self.get_YAML_data(yaml_file_path)['resolution'])

        self.samples = None
        self.hotspot_means = None
        self.hotspot_covs = None
        '''Set the right sigma interval so that majority of the
           samples that are generated with hotspots are within the given bounds'''
        self.sigma_interval = 2.0

        self.get_hotspot_info()
        self.load_charging_positions()

    def load_charging_positions(self):
        path, name = os.path.split(self.map_file_path)
        filepath = path + "/config/Testing/" + os.path.splitext(name)[0] + "_ChargingStations.csv"
        file_exists = os.path.isfile(filepath)
        assert file_exists, "Charging station info file %s does not exist" % filepath
        if file_exists:
            df = pd.read_csv(filepath, sep=',')
            self.charging_positions = df.values

    def get_hotspot_info(self):
        path, name = os.path.split(self.map_file_path)
        filepath = path + "/config/Testing/" + os.path.splitext(name)[0] + "_Hotspots.txt"
        file_exists = os.path.isfile(filepath)
        assert file_exists, "Hotspot file %s does not exist" % filepath
        if file_exists:
            data = np.loadtxt(filepath, delimiter=',')
            self.hotspot_means = data[:, 0:2]
            self.hotspot_covs = np.zeros((data.shape[0], 2, 2))
            self.hotspot_covs[:, 0, 0] = (data[:, 2]/self.sigma_interval)**2
            self.hotspot_covs[:, 1, 1] = (data[:, 3]/self.sigma_interval)**2

            # Extract the source means and cov into different variables
            # The first mean and cov in the hotspot is used as the source hotspot
            self.source_mean = self.hotspot_means[0,:]
            self.hotspot_means = self.hotspot_means[1:, :]
            self.source_cov = self.hotspot_covs[0, :, :]
            self.hotspot_covs= self.hotspot_covs[1:, :, :]

    def get_YAML_data(self, filepath):
        data = None
        with open(filepath, 'r') as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        return data

    def get_color_vector(self, pos):
        # Inverted pos indices because image is given as height x width
        # Also used just the RGB channels.
        return self.img[int(pos[1]), int(pos[0]), 0:3]

    def close_to_obstacles(self, pos):
        collides_with_obstacle = False

        box_half_width = self.robot_radius
        
        min_x = int(max(0, pos[0] - box_half_width))
        max_x = int(min(self.img_width-1, pos[0] + box_half_width))
        min_y = int(max(0, pos[1] - box_half_width))
        max_y = int(min(self.img_height-1, pos[1] + box_half_width))

        # max to min iterations since probability of collision is higher
        # at the borders of the bounding box than at the center
        for i in range(max_x, min_x, -1):
            for j in range(max_y, min_y, -1):
                p = np.array([i, j])
                if np.allclose(np.linalg.norm(self.get_color_vector(p)), 0.0):
                    collides_with_obstacle = True
                    break

        return collides_with_obstacle

    def discard_samples_near_obstacle(self, samples, maxNumSamples, print_progress=False):
        filtered_sample_indices = []
        for s_id in range(samples.shape[0]):
            pos = samples[s_id]
            # Check if point lies on the obstacles
            if np.linalg.norm(self.get_color_vector(pos)) > 0.0:
                # Check if point is close to the obstacle
                if not self.close_to_obstacles(pos):
                    filtered_sample_indices.append(s_id)
                    # Periodically print number of samples generated
                    if print_progress and (len(filtered_sample_indices) % 100) == 0:
                        print("\tGenerated", len(filtered_sample_indices), "samples")
                    # Check if we obtained the max required number of samples
                    if len(filtered_sample_indices) >= maxNumSamples:
                        break

        return samples[filtered_sample_indices,:]

    def discard_outside_map_samples(self, samples):
        filtered_sample_indices = []
        for s_id in range(samples.shape[0]):
            pos = samples[s_id]
            if (pos[0] > 0) and (pos[0] < self.img_width) and \
               (pos[1] > 0) and (pos[1] < self.img_height):
                filtered_sample_indices.append(s_id)

        return samples[filtered_sample_indices,:]

    def get_bivariate_samples(self, nSamples, mean, cov):
        samples = None
        while (samples is None) or (samples.shape[0] < nSamples):
            requiredSamples = nSamples if samples is None else (nSamples - samples.shape[0])
            # Oversample to account for samples that will discarded due to obstacles
            overSampledSize = int(requiredSamples * self.oversamplingFactor)
            newSamples = np.zeros((overSampledSize, 3))
            newSamples[:, 0:2] = np.random.multivariate_normal(mean, cov, overSampledSize)
            newSamples[:,2] = np.random.uniform(-np.pi, np.pi, overSampledSize)
            newSamples = self.discard_outside_map_samples(newSamples)
            newSamples = self.discard_samples_near_obstacle(newSamples, requiredSamples)
            if samples is None:
                samples = newSamples
            else:
                samples = np.vstack((samples, newSamples))

        assert(samples.shape[0] == nSamples)
        return samples

    def generate_focussed_samples(self):
        # Ensure that the number of means and covariances is same
        nMeans = self.hotspot_means.shape[0]
        nCov = self.hotspot_covs.shape[0]
        assert(nMeans == nCov)

        # Ensure that the source mean and covariance is loaded
        assert((self.source_cov is not None) and (self.source_mean is not None))

        # Get samples for source positions
        self.source_samples = self.get_bivariate_samples(self.nRobots, self.source_mean, self.source_cov)
        # Set the resolution
        self.source_samples[:, 0:2] = self.source_samples[:, 0:2] * self.resolution
        print("\tGenerated", self.source_samples.shape[0], "samples for source hotspot")

        # Divide all samples equally among the different target hotspot centers
        sampleSize = int(float(self.nRobots) / nMeans)

        for i in range(nMeans):
            size = sampleSize if (i != (nMeans-1)) else (self.nRobots - (sampleSize * (nMeans - 1)))
            newSamples = self.get_bivariate_samples(size, self.hotspot_means[i], self.hotspot_covs[i])
            if self.samples is None:
                self.samples = newSamples
            else:
                self.samples = np.vstack((self.samples, newSamples))
            print("\tGenerated", newSamples.shape[0], "samples for target hotspot", i+1, "of", nMeans)

        # Set the resolution
        self.samples[:, 0:2] = self.samples[:, 0:2] * self.resolution

        print("Discarded", self.nRobots - self.samples.shape[0], "samples as they were on/close to obstacles")

    def generate_problem_scenarios(self):
        assert(self.charging_positions.shape[0] >= self.nRobots)
        assert(self.source_samples.shape[0] == self.nRobots)
        assert(self.samples.shape[0] == self.nRobots)

        self.problems = np.arange(0, self.nRobots, 1).astype(int)
        np.random.shuffle(self.problems)

    def save_dataset_to_file(self, file_path=None):
        print("\nSaving dataset to a file...")

        if file_path is None:
            file_path = self.root_dir + "/generated/testingData/" + self.map_name +\
                    "-" + str(self.problems.shape[0]) + "Problems.txt"

        height = self.img_height * self.resolution

        nProblems = self.problems.shape[0]
        pose_names = ["C_" + str(i) for i in range(nProblems)]
        pose_names.extend(["S_" + str(i) for i in range(nProblems)])
        pose_names.extend(["T_" + str(i) for i in range(nProblems)])

        source_positions = self.source_samples[self.problems, :]
        target_positions = self.samples[self.problems, :]

        x_pos = self.charging_positions[:self.nRobots, 0].astype(float).tolist()
        y_pos = (height - self.charging_positions[:self.nRobots, 1]).astype(float).tolist()
        theta = self.charging_positions[:self.nRobots, 2].astype(float).tolist()

        x_pos.extend(source_positions[:, 0].astype(float).tolist())
        y_pos.extend((height - source_positions[:, 1]).astype(float).tolist())
        theta.extend(source_positions[:, 2].astype(float).tolist())

        x_pos.extend(target_positions[:, 0].astype(float).tolist())
        y_pos.extend((height - target_positions[:, 1]).astype(float).tolist())
        theta.extend(target_positions[:, 2].astype(float).tolist())

        headers = ["Pose_name", "X", "Y", "Theta"]
        indices = np.arange(0, len(x_pos), 1)

        df = pd.DataFrame(index=indices, columns=headers)
        df = df.fillna("-")
        df["Pose_name"] = pose_names
        df["X"] = x_pos
        df["Y"] = y_pos
        df["Theta"] = theta

        print(df)
        df.to_csv(file_path, sep="\t", header=False, index=False)
        print("Saved generated dataset at", file_path)

    def plot_map(self, ax):
        ax.imshow(self.img)

    def get_cov_ellipse(self, cov, centre, nstd, col='r', **kwargs):
        '''Source of this snippet for plotting ellipses: 
        https://scipython.com/book/chapter-7-matplotlib/examples/bmi-data-with-confidence-ellipses/
        '''
        # Find and sort eigenvalues and eigenvectors into descending order
        eigvals, eigvecs = np.linalg.eigh(cov)
        order = eigvals.argsort()[::-1]
        eigvals, eigvecs = eigvals[order], eigvecs[:, order]

        # The anti-clockwise angle to rotate our ellipse by 
        vx, vy = eigvecs[:,0][0], eigvecs[:,0][1]
        theta = np.arctan2(vy, vx)

        # Width and height of ellipse to draw
        width, height = 2 * nstd * np.sqrt(eigvals)
        ell= Ellipse(xy=centre, width=width, height=height,
                       angle=np.degrees(theta), linewidth=5, **kwargs)
        ell.set_facecolor('none')
        ell.set_edgecolor(col)
        return ell

    def plot_samples(self, ax, filter_samples=False):
        if filter_samples:
            start_pose_ids = self.problems[:, 0].astype(int)
            goal_pose_ids = self.problems[:, 1].astype(int)

            start_sample_poses = self.samples[start_pose_ids, :]
            goal_sample_poses = self.samples[goal_pose_ids, :]
            poses = np.vstack((start_sample_poses, goal_sample_poses))
        else:
            poses = self.samples
            poses = np.vstack((poses, self.source_samples))
            poses = np.vstack((poses, self.charging_positions))

        thetas = poses[:, 2]
        poses = poses / self.resolution

        # plot each individual poses as a point
        ax.scatter(poses[:, 0], poses[:,1], s=self.robot_radius*2.0)

        # plot arrows corresponding to the theta value associated with each pose
        for i in range(thetas.shape[0]):
            ax.arrow(poses[i, 0], poses[i,1],
                    self.robot_radius*np.cos(thetas[i]), self.robot_radius*np.sin(thetas[i]), length_includes_head=True,
                    head_width=self.robot_radius/3.0, head_length=self.robot_radius/3.0, fc='k', ec='k')

    def plot_problems(self, ax):
        for i, p in enumerate(self.problems):
            charging = self.charging_positions[i]
            source = self.source_samples[p]
            target = self.samples[p]
            x = np.array([charging[0], source[0], target[0], charging[0]]) / self.resolution
            y = np.array([charging[1], source[1], target[1], charging[1]]) / self.resolution
            plt.plot(x, y)

    def save_debug_map(self, file_path=None, plot_problems=True, plot_ellipses=True):
        print("\nGenerating debug map...")
        f = plt.figure(figsize=(20, 20))
        ax = f.subplots()

        self.plot_map(ax)
        self.plot_samples(ax)

        if plot_problems:
            self.plot_problems(ax)

        if plot_ellipses and self.hotspot_covs is not None:
            for i in range(self.hotspot_covs.shape[0]):
                ax.add_artist(self.get_cov_ellipse(self.hotspot_covs[i],
                 self.hotspot_means[i], self.sigma_interval))
            ax.add_artist(self.get_cov_ellipse(self.source_cov,
                 self.source_mean, self.sigma_interval, col='b'))

        if file_path is None:
            file_path = self.root_dir + "/generated/testingData/debugMaps/" + self.map_name +\
                        "-" + str(self.nRobots) + "Problems.svg"
        plt.savefig(file_path, format='svg')
        print("Saved debug map at", file_path)

    def generate_dataset(self):
        for n in self.nRobotsList:
            self.nRobots = n
            self.source_samples = None
            self.samples = None
            print("\n========= Generating Testing Dataset ==========")
            print("Map:\t\t\t", self.map_filename)
            print("Num of robots:\t\t", self.nRobots)
            print("Robot radius:\t\t", self.robot_radius)
            print("Oversampling rate:\t", self.oversamplingFactor)
            print("------------------------------------------------")

            if (self.nRobots <= self.charging_positions.shape[0]):
                print("Requested to place", n, "robots for testing.")
                print("Generating samples for source and target locations...")
                self.generate_focussed_samples()
                print("Successfully generated", self.source_samples.shape[0],\
                    "samples for source locations and", self.samples.shape[0], "samples for target locations")

                print("\nGenerating", self.nRobots, "unique problems from generated samples...")
                self.generate_problem_scenarios()
                print("Successfully generated", self.problems.shape[0], "problems")

                self.save_dataset_to_file()
                if self.save_dbg_image:
                    self.save_debug_map()
            else:
                print("Number of robots must be less than the number of"
                    " charging poses available! This map has a maximum of", self.charging_positions.shape[0], "charging positions")
            print("================================================")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_filename", type=str, help="Filename of the map image that should be used for dataset generation (ex. map1.png)")
    parser.add_argument("--nRobots", required=True, nargs="*", type=int, help="Number of robots to be used for testing", default=[10])
    parser.add_argument("--robot_radius", type=int, help="Radius of the robot (in pixels) to be used for collision detection", default=10)
    parser.add_argument("--oversampling", type=float, help="Oversampling factor so to account for samples that will discarded due to their proximity to obstacles. (Default=4.0)", default=4.0)
    parser.add_argument("--dbg_image", type=bool, help="Generate a debug image to visualize generated dataset (Disabled by default)", default=False)
    args = parser.parse_args()

    data_gen = DatasetGenerator(args)
    data_gen.generate_dataset()

if __name__ == "__main__":
    main()
