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
from matplotlib import cm

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



class PlaneDetection():
    def __init__(self, point_cloud):

        self.point_cloud = point_cloud

    def colorizeInliers(self, r,g,b):
        self.inlier_cloud.paint_uniform_color([r,g,b]) # paints the plane in red

    def segment(self, distance_threshold=0.25, ransac_n=3, num_iterations=50):

        print('Starting plane detection')
        plane_model, inlier_idxs = self.point_cloud.segment_plane(distance_threshold=distance_threshold, 
                                                    ransac_n=ransac_n,
                                                    num_iterations=num_iterations)
        [self.a, self.b, self.c, self.d] = plane_model

        self.inlier_cloud = self.point_cloud.select_by_index(inlier_idxs)

        outlier_cloud = self.point_cloud.select_by_index(inlier_idxs, invert=True)

        return outlier_cloud

    def __str__(self):
        text = 'Segmented plane from pc with ' + str(len(self.point_cloud.points)) + ' with ' + str(len(self.inlier_cloud.points)) + ' inliers. '
        text += '\nPlane: ' + str(self.a) +  ' x + ' + str(self.b) + ' y + ' + str(self.c) + ' z + ' + str(self.d) + ' = 0' 
        return text


def main():

    # ------------------------------------------
    # Initialization
    # ------------------------------------------
    print("Load a ply point cloud, print it, and render it")
    point_cloud_original = o3d.io.read_point_cloud('../Ex2/factory_without_ground.ply')


    number_of_planes = 6
    colormap = cm.Pastel1(list(range(0,number_of_planes)))

    # ------------------------------------------
    # Execution
    # ------------------------------------------

    point_cloud = deepcopy(point_cloud_original) 
    planes = []
    while True: # run consecutive plane detections

        plane = PlaneDetection(point_cloud) # create a new plane instance
        point_cloud = plane.segment() # new point cloud are the outliers of this plane detection
        print(plane)

        # colorization using a colormap
        idx_color = len(planes)
        color = colormap[idx_color, 0:3]
        plane.colorizeInliers(r=color[0], g=color[1], b=color[2])

        planes.append(plane)

        if len(planes) >= number_of_planes: # stop detection planes
            break


    # ------------------------------------------
    # Visualization
    # ------------------------------------------

    # Create a list of entities to draw
    entities = [x.inlier_cloud for x in planes]
    entities.append(point_cloud)

    o3d.visualization.draw_geometries(entities,
                                    zoom=view['trajectory'][0]['zoom'],
                                    front=view['trajectory'][0]['front'],
                                    lookat=view['trajectory'][0]['lookat'],
                                    up=view['trajectory'][0]['up'])

if __name__ == "__main__":
    main()
