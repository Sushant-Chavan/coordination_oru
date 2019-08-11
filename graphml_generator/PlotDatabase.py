#!/usr/bin/env python
# coding: utf-8

import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import argparse
import glob
import subprocess
import sys
import os
import yaml
import pandas as pd

def get_node_list(graph):
    nodes = []
    for i in range(len(graph.nodes)):
        nodes.append('n' + str(i))
    return nodes

def get_edge_list(graph):
    return sorted(list(graph.edges))

def get_node_data_as_list(graph, nodeID):
    n = graph.nodes[nodeID]
    c = [float(i) for i in n['coords'].split(',')]
    return c

def get_node_data(graph):
    ''' Return the node coordinates of all the nodes in the graph as a matrix'''

    data = None

    for n in get_node_list(graph):
        if data is None:
            data = np.array(get_node_data_as_list(graph, n))
        else:
            data = np.vstack((data, get_node_data_as_list(graph, n)))
    return data

def plot_Thunder_graph(graph_path, map_filename, output_filename, output_format, nExperiences, resolution_multiplier=10, plot_node_ids=True):
    img = plt.imread(map_filename)
    img_height = img.shape[0]
    img_width = img.shape[1]

    f = plt.figure(figsize=plt.figaspect(img_height/float(img_width))*3.0)
    ax = f.subplots()
    ax.imshow(img)

    for filename in glob.iglob(graph_path + '/*.graphml', recursive=True):
        graph = nx.read_graphml(filename)

        nodes = get_node_data(graph)
        edges = get_edge_list(graph)

        # Discard the theta values
        nodes = nodes[:, 0:2]

        ax.scatter(nodes[:,0] * resolution_multiplier, img_height - (nodes[:,1] * resolution_multiplier), s=100/resolution_multiplier)

        node_list = get_node_list(graph)
        node_ids_map = {}
        for i in range(len(node_list)):
            node_ids_map[node_list[i]] = i
            if plot_node_ids:
                ax.text(nodes[i,0] * resolution_multiplier, img_height - (nodes[i,1] * resolution_multiplier), node_list[i], size=50/resolution_multiplier)

        for e in edges:
            start = node_ids_map[e[0]]
            end = node_ids_map[e[1]]

            x = np.array([nodes[start,0], nodes[end,0]]) * resolution_multiplier
            y = img_height - np.array([nodes[start,1], nodes[end,1]]) * resolution_multiplier

            ax.plot(x, y, linewidth=20/resolution_multiplier)

    ax.set_xticklabels([])
    ax.set_yticklabels([])

    ax.set_title("Thunder database with " + str(nExperiences) + " robot experiences")
    plt.savefig(output_filename, format=output_format, bbox_inches='tight')

def plot_Lightning_graph(graph_path, map_filename, output_filename, output_format, nExperiences, resolution_multiplier=10):
    img = plt.imread(map_filename)
    img_height = img.shape[0]
    img_width = img.shape[1]

    f = plt.figure(figsize=plt.figaspect(img_height/float(img_width))*3.0)
    ax = f.subplots()
    ax.imshow(img)

    for filename in glob.iglob(graph_path + '/*.graphml', recursive=True):
        graph = nx.read_graphml(filename)

        nodes = get_node_data(graph)

        nodes = nodes[:, 0:2]

        ax.scatter(nodes[:,0] * resolution_multiplier, img_height - (nodes[:,1] * resolution_multiplier), s=100/resolution_multiplier)
        ax.plot(nodes[:,0] * resolution_multiplier, img_height - (nodes[:,1] * resolution_multiplier), linewidth=10/resolution_multiplier)

        # Render node ID's
        # node_list = get_node_list(graph)
        # for i in range(len(node_list)):
        #     ax.text(nodes[i,0] * resolution_multiplier, img_height - (nodes[i,1] * resolution_multiplier), node_list[i])

    ax.set_xticklabels([])
    ax.set_yticklabels([])

    ax.set_title("Lightning database with " + str(nExperiences) + " robot experiences")
    plt.savefig(output_filename, format=output_format, bbox_inches='tight')

def plot_Egraph(graph_path, map_filename, output_filename, output_format, nExperiences, resolution_multiplier=10):
    img = plt.imread(map_filename)
    img_height = img.shape[0]
    img_width = img.shape[1]
    step_size = 0.2

    f = plt.figure(figsize=plt.figaspect(img_height/float(img_width))*3.0)
    ax = f.subplots()

    ax.imshow(img, interpolation='nearest', aspect='auto')

    for filename in glob.iglob(graph_path + '/*.csv', recursive=True):
        df = pd.read_csv(filename)

        # Do the conversion from RobotState to RobotCoord and backwards 
        # according to the SMPL ManipLattice to dicretize the data for plotting
        nodes = (((df.values[:, 0:2] / 0.2) + 0.5).astype(int) * 0.2).astype(float)

        ax.scatter((nodes[:,0] * resolution_multiplier), (img_height - (nodes[:,1] * resolution_multiplier)), s=10/resolution_multiplier, color='r')
        ax.plot(nodes[:,0] * resolution_multiplier, img_height - (nodes[:,1] * resolution_multiplier), linewidth=25/resolution_multiplier)

    dx = max(1.0, int(round((img_width /resolution_multiplier)/step_size)))
    dy = max(1.0, int(round((img_height /resolution_multiplier)/step_size)))

    x_ticks = np.linspace(0, img_width+1, dx)
    y_ticks = np.linspace(0, img_height+1, dy)

    # xx, yy = np.meshgrid(x_ticks, y_ticks)
    # plt.scatter(xx, yy, facecolors='none', edgecolors='k', s=10/resolution_multiplier)

    ax.set_xticks(x_ticks)
    ax.set_yticks(y_ticks)

    # Show the major grid lines with dark grey linesc to demostrate all possible nodes in the graph
    ax.grid(b=True, which='major', color='#000000', linestyle='-', alpha=0.1)
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    ax.set_title("EGraphs database with " + str(nExperiences) + " robot experiences")
    plt.savefig(output_filename, format=output_format, bbox_inches='tight')

def get_YAML_data(filepath):
    data = None
    with open(filepath, 'r') as stream:
        try:
            data = yaml.safe_load(stream)
            print("Loaded YAML data from file", filepath)
        except yaml.YAMLError as exc:
            print(exc)
    return data

def get_database_filepath(args):
    sampling_name = "Uniform" if args.no_hotspots else "UsingHotspots"
    kinematics = "ReedsSheep" if args.non_holonomic else "Holonomic"
    db_name = "Thunder.db" if args.thunder else "Lightning.db"
    db_name = "EGraphs" if args.egraph else db_name
    map_name = os.path.splitext(args.map_image_filename)[0]
    directory = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../generated/experienceDBs/")
    directory = os.path.join(directory, map_name)
    directory = os.path.join(directory, str(args.count)+"_TrainingExperiences")
    directory = os.path.join(directory, sampling_name)
    directory = os.path.join(directory, kinematics)
    path = os.path.join(directory, db_name)
    return path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--experience_db_filename", help="Filename of the experience database file (ex. thunder.db)")
    parser.add_argument("--map_image_filename", help="Filename of the map image that was used for generating the experience database (ex. map1.png)", default="map1.png")
    parser.add_argument("--output_filename", help="File name of the output file (ex. plot.svg)", default=None)
    parser.add_argument("--thunder", help="Set this if plotting Thunder DB", action="store_true", default=False)
    parser.add_argument("--egraph", help="Set this if plotting Egraphs DB", action="store_true", default=False)
    parser.add_argument("--output_format", help="Fileformat (default svg) for output image (png/svg).", default='svg')
    parser.add_argument("--turning_radius", help="Turning Radius of the ReedsShep cars", default=4.0)
    parser.add_argument("--non_holonomic", help="Set if the robot is non-holonomic (default: False)", action="store_true", default=False)
    parser.add_argument("--count", type=int, help="Number of problems present in the training dataset. Default: 10", default=10)
    parser.add_argument("--no_hotspots", type=bool, help="Indicate if the experience database is being generated using uniform sampling of the map. Default: False (hotspots used)", default=False)
    args = parser.parse_args()

    dir_name, _ = os.path.split(os.path.abspath(sys.argv[0]))
    graph_files_output_dir = os.path.abspath(dir_name + "/../generated/graphFiles")

    turning_radius = args.turning_radius
    is_thunder = args.thunder
    holonomic = not args.non_holonomic
    experience_db_path = get_database_filepath(args)
    map_file_path = os.path.abspath(dir_name + "/../maps/" + args.map_image_filename)
    yaml_file_path = os.path.splitext(map_file_path)[0] + ".yaml"
    resolution_multiplier = float(1.0/get_YAML_data(yaml_file_path)['resolution'])

    plot_file_path = os.path.splitext(experience_db_path)[0] + ".svg"

    plot_output_format = args.output_format

    if not args.egraph:
        if not os.path.isfile(experience_db_path):
            print("Experience Database Specified Does Not Exist! \nPath specified was:\n", experience_db_path)
            return
        if not os.path.isfile(map_file_path):
            print("The Map File Specified Does Not Exist! \nPath specified was:\n", map_file_path)
            return

        subprocess.call([dir_name + "/build/GenerateGraphml", 
                        str(turning_radius),
                        '1' if is_thunder else "0",
                        experience_db_path,
                        map_file_path,
                        str(1.0/resolution_multiplier),
                        graph_files_output_dir,
                        '1' if holonomic else "0"])

    print("\nPlotting the Graphml file contents onto the map")
    if is_thunder:
        plot_Thunder_graph(graph_files_output_dir + "/", map_file_path, plot_file_path, plot_output_format, args.count, resolution_multiplier)
    elif args.egraph:
        plot_Egraph(experience_db_path, map_file_path, plot_file_path, plot_output_format, args.count, resolution_multiplier)
    else:
        plot_Lightning_graph(graph_files_output_dir + "/", map_file_path, plot_file_path, plot_output_format, args.count, resolution_multiplier)

    print("Plot successfully generated and can be found at:\n", plot_file_path)


if __name__ == "__main__":
    main()
