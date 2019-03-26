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

def get_node_list(graph):
    return sorted(list(graph.nodes))

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

def plot_Thunder_graph(graph_path, map_filename, output_filename, output_format):
    img = plt.imread(map_filename)
    img_height = img.shape[0]
    img_width = img.shape[1]

    f = plt.figure(figsize=(20, 20))
    ax = f.subplots()
    ax.imshow(img)

    for filename in glob.iglob(graph_path + '/*.graphml', recursive=True):
        graph = nx.read_graphml(filename)

        nodes = get_node_data(graph)
        edges = get_edge_list(graph)

        # Discard the theta values
        nodes = nodes[:, 0:2]

        ax.scatter(nodes[:,0] * 10, img_height - (nodes[:,1] * 10))

        node_list = get_node_list(graph)
        node_ids_map = {}
        for i in range(len(node_list)):
            node_ids_map[node_list[i]] = i
            ax.text(nodes[i,0] * 10, img_height - (nodes[i,1] * 10), node_list[i])

        for e in edges:
            start = node_ids_map[e[0]]
            end = node_ids_map[e[1]]

            x = np.array([nodes[start,0], nodes[end,0]]) * 10
            y = img_height - np.array([nodes[start,1], nodes[end,1]]) * 10

            ax.plot(x, y)

    plt.savefig(output_filename, format=output_format)

def plot_Lightning_graph(graph_path, map_filename, output_filename, output_format):
    img = plt.imread(map_filename)
    img_height = img.shape[0]
    img_width = img.shape[1]

    f = plt.figure(figsize=(20, 20))
    ax = f.subplots()
    ax.imshow(img)

    for filename in glob.iglob(graph_path + '/*.graphml', recursive=True):
        graph = nx.read_graphml(filename)

        nodes = get_node_data(graph)

        nodes = nodes[:, 0:2]

        ax.scatter(nodes[:,0] * 10, img_height - (nodes[:,1] * 10))
        ax.plot(nodes[:,0] * 10, img_height - (nodes[:,1] * 10))

        # Render node ID's
        # node_list = get_node_list(graph)
        # for i in range(len(node_list)):
        #     ax.text(nodes[i,0] * 10, img_height - (nodes[i,1] * 10), node_list[i])

    plt.savefig(output_filename, format=output_format)

def get_experience_db_path(user_db_filepath, map_image_filename, is_thunder, root_dir):
    db_filename =  user_db_filepath if user_db_filepath is not None \
        else os.path.splitext(map_image_filename)[0] + "_" + ("thunder.db" if is_thunder else "lightning.db")
    db_file_path = os.path.abspath(root_dir + "/../generated/experienceDBs/" + db_filename)
    return db_file_path

def get_plot_filepath(user_output_filename, map_image_filename, is_thunder, root_dir):
    plot_filename =  user_output_filename if user_output_filename is not None \
        else os.path.splitext(map_image_filename)[0] + "_" + ("thunder.svg" if is_thunder else "lightning.svg")
    plot_file_path = os.path.abspath(root_dir + "/../generated/plots/" + plot_filename)
    return plot_file_path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--experience_db_filename", help="Filename of the experience database file (ex. thunder.db)")
    parser.add_argument("--map_image_filename", help="Filename of the map image that was used for generating the experience database (ex. map1.png)", default="map1.png")
    parser.add_argument("--output_filename", help="File name of the output file (ex. plot.svg)", default=None)
    parser.add_argument("--is_thunder_db", help="Set this if plotting Thunder DB", action="store_true", default=False)
    parser.add_argument("--output_format", help="Fileformat (default svg) for output image (png/svg).", default='svg')
    parser.add_argument("--turning_radius", help="Turning Radius of the ReedsShep cars", default=4.0)
    parser.add_argument("--map_resolution", help="Map_resolution", default=0.1)
    args = parser.parse_args()

    dir_name, _ = os.path.split(os.path.abspath(sys.argv[0]))
    graph_files_output_dir = os.path.abspath(dir_name + "/../generated/graphFiles")

    turning_radius = args.turning_radius
    is_thunder = args.is_thunder_db
    experience_db_path = get_experience_db_path(args.experience_db_filename, args.map_image_filename, is_thunder, dir_name)
    map_file_path = os.path.abspath(dir_name + "/../maps/" + args.map_image_filename)
    map_resolution = args.map_resolution

    plot_file_path = get_plot_filepath(args.output_filename, args.map_image_filename, is_thunder, dir_name)

    plot_output_format = args.output_format

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
                     str(map_resolution),
                     graph_files_output_dir])

    print("\nPlotting the Graphml file contents onto the map")
    if is_thunder:
        plot_Thunder_graph(graph_files_output_dir + "/", map_file_path, plot_file_path, plot_output_format)
    else:
        plot_Lightning_graph(graph_files_output_dir + "/", map_file_path, plot_file_path, plot_output_format)

    print("Plot successfully generated and can be found at:\n", plot_file_path)


if __name__ == "__main__":
    main()
