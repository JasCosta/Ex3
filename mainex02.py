#!/usr/bin/env python3

import csv
import pickle
from copy import deepcopy
from random import randint
from turtle import color

import open3d as o3d
import cv2
import numpy as np
import matplotlib.pyplot as plt

view = {
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : False,
	"trajectory" : 
	
			
[
		{
		
	
			"boundingbox_max" : [ 0.87671953439712524, 0.4845428466796875, 2.8718910217285156 ],
			"boundingbox_min" : [ -1.5309394598007202, -2.0348093509674072, 0.50720000267028809 ],
			"field_of_view" : 60.0,
			"front" : [ -0.41460237676951595, -0.0025070903385497902, -0.90999922179917436 ],
			"lookat" : [ 1.137925279869632, -0.12370299296685888, 5.2201208842767555 ],
			"up" : [ 0.28074564666193819, -0.9515696270869648, -0.12528817456545938 ],
			"zoom" : 1.5799999999999996
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}


def main():

    # ------------------------------------------
    # Initialization
    # ------------------------------------------
    print("Load a ply point cloud, print it, and render it")
    point_cloud = o3d.io.read_point_cloud('../data/Factory/13.ply')

    # ------------------------------------------
    # Execution
    # ------------------------------------------

    print('Starting plane detection')
    plane_model, inlier_idxs = point_cloud.segment_plane(distance_threshold=0.02, 
                                                    ransac_n=3,
                                                    num_iterations=100)
    [a, b, c, d] = plane_model
    print('Plane equation: ' + str(a) +  ' x + ' + str(b) + ' y + ' + str(c) + ' z + ' + str(d) + ' = 0' )

    inlier_cloud = point_cloud.select_by_index(inlier_idxs)
    inlier_cloud.paint_uniform_color([1.0, 0, 0]) # paints the plane in red
    outlier_cloud = point_cloud.select_by_index(inlier_idxs, invert=True)

    # ------------------------------------------
    # Visualization
    # ------------------------------------------
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                    zoom=view['trajectory'][0]['zoom'],
                                    front=view['trajectory'][0]['front'],
                                    lookat=view['trajectory'][0]['lookat'],
                                    up=view['trajectory'][0]['up'])

    o3d.io.write_point_cloud('./factory_without_ground.ply', outlier_cloud, write_ascii=False, compressed=False, print_progress=False)

if __name__ == "__main__":
    main()
