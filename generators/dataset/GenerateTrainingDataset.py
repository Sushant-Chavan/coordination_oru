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
import errno
import yaml
import time
import pandas as pd
import glob

class DatasetGenerator():
    def __init__(self, args):
        self.root_dir = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../")
        self.map_filename = args.map_filename
        self.nProblemsList = args.nProblems
        self.robot_radius = int(args.robot_radius)
        self.oversamplingFactor = args.oversampling
        self.save_dbg_image = args.dbg_image

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

        if args.use_hotspots:
            self.get_hotspot_info()

    def get_hotspot_info(self):
        path, name = os.path.split(self.map_file_path)
        filepath = path + "/config/Training/" + os.path.splitext(name)[0] + "_Hotspots.txt"
        file_exists = os.path.isfile(filepath)
        assert file_exists, "Hotspot file %s does not exist" % filepath
        if file_exists:
            data = np.loadtxt(filepath, delimiter=',')
            self.hotspot_means = data[:, 0:2]
            self.hotspot_covs = np.zeros((data.shape[0], 2, 2))
            self.hotspot_covs[:, 0, 0] = (data[:, 2]/self.sigma_interval)**2
            self.hotspot_covs[:, 1, 1] = (data[:, 3]/self.sigma_interval)**2

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

    def load_previously_generated_samples(self):
        directory = self.get_or_create_dir()
        files = [f for f in glob.glob(directory + '/' + self.map_filename.split('.')[0]+'*.txt')]

        highest_exp_idx = None
        highest_exp = None
        for i, f in enumerate(files):
            count = f.split('-')[-1]
            count = int(count.split('P')[0])

            if highest_exp is None or count > highest_exp:
                highest_exp = count
                highest_exp_idx = i
        
        df = pd.read_csv(files[highest_exp_idx], header=None, sep='\t', usecols=[1,2,3])
        data = df.values
        nProblems = int(data.shape[0]/2)
        start_training_poses = data[:nProblems, :]
        goal_training_poses = data[nProblems:, :]

        return start_training_poses, goal_training_poses


    def generate_random_samples(self, nSamples):
        # Oversample to account for samples that will discarded due to obstacles
        sampleSize = int(nSamples * self.oversamplingFactor)

        self.samples = np.zeros((sampleSize, 3))
        self.samples[:,0] = np.random.randint(0, self.img_width, self.samples.shape[0])
        self.samples[:,1] = np.random.randint(0, self.img_height, self.samples.shape[0])
        self.samples[:,2] = np.random.uniform(-np.pi, np.pi, self.samples.shape[0])
        self.samples = self.discard_samples_near_obstacle(self.samples, nSamples, print_progress=True)

        # Set the resolution
        self.samples[:, 0:2] = self.samples[:, 0:2] * self.resolution

        print("Discarded", nSamples - self.samples.shape[0], "samples as they were on/close to obstacles")

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

    def generate_focussed_samples(self, nSamples):
        # Ensure that the number of means and covariances is same
        nMeans = self.hotspot_means.shape[0]
        nCov = self.hotspot_covs.shape[0]
        assert(nMeans == nCov)

        # Divide all samples equallly among the different hotspot centers
        sampleSize = int(float(nSamples) / nMeans)
        print("Sample Size:", sampleSize)

        if sampleSize > 0:
            extra_samples = nSamples % nMeans
            for i in range(nMeans):
                size = (sampleSize + 1) if (i < extra_samples) else sampleSize
                newSamples = self.get_bivariate_samples(size, self.hotspot_means[i], self.hotspot_covs[i])
                if self.samples is None:
                    self.samples = newSamples
                else:
                    self.samples = np.vstack((self.samples, newSamples))
                print("\tGenerated", newSamples.shape[0], "samples for hotspot", i+1, "of", nMeans)
        else:
            for i in range(nSamples):
                newSamples = self.get_bivariate_samples(1, self.hotspot_means[i], self.hotspot_covs[i])
                if self.samples is None:
                    self.samples = newSamples
                else:
                    self.samples = np.vstack((self.samples, newSamples))
                print("\tGenerated", newSamples.shape[0], "samples for hotspot", i+1, "of", nMeans)

        # Set the resolution
        self.samples[:, 0:2] = self.samples[:, 0:2] * self.resolution

        print("Discarded", nSamples - self.samples.shape[0], "samples as they were on/close to obstacles")

    def generate_problem_scenarios(self):
        nSamples = self.samples.shape[0]

        if nSamples < (self.nProblems * 2):
            print("Insufficient number of samples generated! Min number of samples = 2 * nProblems")
            print("Try increasing the oversampling factor to generate more samples")
            return False

        used_samples = np.full((1, nSamples), False)

        self.problems = np.zeros((self.nProblems, 2))

        for p_idx in range(self.nProblems):
            start = 0
            goal = 0

            # Find a unique start position
            while used_samples[0,start]:
                start = np.random.randint(0, nSamples, 1)[0]
            used_samples[0, start] = True
                
            # Find a unique goal position
            while used_samples[0,goal]:
                goal = np.random.randint(0, nSamples, 1)[0] 
            used_samples[0, goal] = True

            self.problems[p_idx, 0] = start
            self.problems[p_idx, 1] = goal

        self.problems = self.problems.astype(int)
        return True

    def get_or_create_dir(self, debugMaps=False):
        strategy = "UniformSampling" if self.hotspot_means is None else "UsingHospots"
        directory = os.path.join(self.root_dir, "generated/trainingData/")
        directory = os.path.join(directory, strategy)
        if debugMaps:
            directory = os.path.join(directory, "debugMaps")

        try:
            os.makedirs(directory)
        except OSError as exc:
            if exc.errno ==errno.EEXIST and os.path.isdir(directory):
                pass
            else:
                raise "Could not create directory {}".format(directory)

        return directory

    def save_dataset_to_file(self, file_path=None):
        print("\nSaving dataset to a file...")

        if file_path is None:
            directory = self.get_or_create_dir()
            filename = self.map_name + "-" + str(self.problems.shape[0]) + "Problems.txt"
            file_path = os.path.join(directory, filename)

        height = self.img_height * self.resolution

        nProblems = self.problems.shape[0]
        pose_names = ["Start_" + str(i) for i in range(nProblems)]
        pose_names.extend(["Goal_" + str(i) for i in range(nProblems)])

        start_pose_ids = self.problems[:, 0].astype(int)
        goal_pose_ids = self.problems[:, 1].astype(int)

        start_sample_poses = self.samples[start_pose_ids, :]
        goal_sample_poses = self.samples[goal_pose_ids, :]

        x_pos = start_sample_poses[:, 0].astype(float).tolist()
        y_pos = (height - start_sample_poses[:, 1]).astype(float).tolist()
        theta = start_sample_poses[:, 2].astype(float).tolist()

        x_pos.extend(goal_sample_poses[:, 0].astype(float).tolist())
        y_pos.extend((height - goal_sample_poses[:, 1]).astype(float).tolist())
        theta.extend(goal_sample_poses[:, 2].astype(float).tolist())

        headers = ["Pose_name", "X", "Y", "Theta"]
        indices = np.arange(0, len(x_pos), 1)

        self.df = pd.DataFrame(index=indices, columns=headers)
        self.df = self.df.fillna("-")
        self.df["Pose_name"] = pose_names
        self.df["X"] = x_pos
        self.df["Y"] = y_pos
        self.df["Theta"] = theta

        self.df = self.replace_with_generated_data(self.df)

        self.df.to_csv(file_path, sep="\t", header=False, index=False)
        print("Saved generated dataset at", file_path)

    def replace_with_generated_data(self, df):
        nProblems = self.problems.shape[0]

        start_poses, end_poses = self.load_previously_generated_samples()
        n_loaded_poses = start_poses.shape[0]

        print(nProblems, n_loaded_poses)
        if nProblems < n_loaded_poses:
            df["X"][0:nProblems] = start_poses[0:nProblems, 0]
            df["Y"][0:nProblems] = start_poses[0:nProblems, 1]
            df["Theta"][0:nProblems] = start_poses[0:nProblems, 2]

            df["X"][nProblems:] = end_poses[0:nProblems, 0]
            df["Y"][nProblems:] = end_poses[0:nProblems, 1]
            df["Theta"][nProblems:] = end_poses[0:nProblems, 2]
        else:
            df["X"][0:n_loaded_poses] = start_poses[:, 0]
            df["Y"][0:n_loaded_poses] = start_poses[:, 1]
            df["Theta"][0:n_loaded_poses] = start_poses[:, 2]

            df["X"][nProblems:nProblems+n_loaded_poses] = end_poses[:, 0]
            df["Y"][nProblems:nProblems+n_loaded_poses] = end_poses[:, 1]
            df["Theta"][nProblems:nProblems+n_loaded_poses] = end_poses[:, 2]

        return df

    def plot_map(self, ax):
        ax.imshow(self.img)

    def get_cov_ellipse(self, cov, centre, nstd, **kwargs):
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
        ell.set_edgecolor('r')
        return ell

    def get_poses_to_polt(self, filter_samples=True):
        if filter_samples:
            height = self.img_height * self.resolution
            start_sample_poses = np.array(self.df.iloc[:self.nProblems, 1:4])
            goal_sample_poses = np.array(self.df.iloc[self.nProblems:, 1:4])
            start_sample_poses[:, 1] = height - start_sample_poses[:, 1]
            goal_sample_poses[:, 1] = height - goal_sample_poses[:, 1]
            poses = np.vstack((start_sample_poses, goal_sample_poses))
        else:
            poses = self.samples

        return poses

    def plot_samples(self, ax, filter_samples=True):
        poses = self.get_poses_to_polt(filter_samples)

        thetas = poses[:, 2]
        poses = poses / self.resolution

        # plot each individual poses as a point
        ax.scatter(poses[:, 0], poses[:,1], s=self.robot_radius/2.0)

        # plot arrows corresponding to the theta value associated with each pose
        for i in range(thetas.shape[0]):
            ax.arrow(poses[i, 0], poses[i,1],
                    self.robot_radius*np.cos(thetas[i]), self.robot_radius*np.sin(thetas[i]),
                    head_width=self.robot_radius/2.0, head_length=self.robot_radius/2.0, fc='k', ec='k')

    def plot_problems(self, ax):
        poses = self.get_poses_to_polt(filter_samples=True)
        start_poses = poses[0:self.nProblems, :]
        goal_poses = poses[self.nProblems:, :]

        for i in range(self.nProblems):
            start = start_poses[i]
            goal = goal_poses[i]
            x = np.array([start[0], goal[0]]) / self.resolution
            y = np.array([start[1], goal[1]]) / self.resolution
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

        if file_path is None:
            directory = self.get_or_create_dir(debugMaps=True)
            filename = self.map_name + "-" + str(self.nProblems) + "Problems.svg"
            file_path = os.path.join(directory, filename)
        plt.savefig(file_path, format='svg')
        print("Saved debug map at", file_path)

    def generate_dataset(self):
        for n in self.nProblemsList:
            self.nProblems = n
            self.samples = None
            print("\n========= Generating Training Dataset ==========")
            print("Map:\t\t\t", self.map_filename)
            print("Num of problems:\t", self.nProblems)
            print("Robot radius:\t\t", self.robot_radius)
            print("Oversampling rate:\t", self.oversamplingFactor)
            print("------------------------------------------------")

            # Ideally we need (nProblems * 2) samples for nProblems.
            nSamples = self.nProblems * 2

            if self.hotspot_means is not None:
                print("Generating", nSamples, "samples at the hotspots...")
                self.generate_focussed_samples(nSamples)
            else:
                # Add some wiggle room for creating problems.
                # We generate more samples than needed to spread out the problems more evenly
                wiggle_factor = 2
                nSamples = nSamples * wiggle_factor
                print("Generating", nSamples, "samples (with wiggle factor of", wiggle_factor, "), uniformly over the map...")
                self.generate_random_samples(nSamples)
            print("Successfully generated", self.samples.shape[0], "samples")

            print("\nGenerating", self.nProblems, "unique problems from generated samples...")
            success = self.generate_problem_scenarios()
            if not success:
                return
            print("Successfully generated", self.problems.shape[0], "problems")
            print("================================================")

            self.save_dataset_to_file()
            if self.save_dbg_image:
                self.save_debug_map()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_filename", type=str, help="Filename of the map image that should be used for dataset generation (ex. map1.png)")
    parser.add_argument("--nProblems", required=True, nargs="*", type=int, help="Number of training problems to be generated (ex. 10 100 1000)", default=[100])
    parser.add_argument("--robot_radius", type=int, help="Radius of the robot (in pixels) to be used for collision detection", default=10)
    parser.add_argument("--oversampling", type=float, help="Oversampling factor so to account for samples that will discarded due to their proximity to obstacles. (Default=4.0)", default=4.0)
    parser.add_argument("--dbg_image", type=bool, help="Generate a debug image to visualize generated dataset (Disabled by default)", default=False)
    parser.add_argument("--use_hotspots", type=bool, help="Flag to activate use of hotspots for dataset generation (Disabled by default)", default=False)
    args = parser.parse_args()

    data_gen = DatasetGenerator(args)
    data_gen.generate_dataset()

if __name__ == "__main__":
    main()
