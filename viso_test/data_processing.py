import open3d as o3d
import numpy as np
import os

filename = "cloud_pillar.ply"
pcd = o3d.io.read_point_cloud(filename)

print("Running RANSAC")
plane_model, inliers = pcd.segment_plane(distance_threshold = 0.05, 
                                         ransac_n = 3, 
                                         num_iterations = 1000)

# Distance Threshold - Max Distance from Plane to be inlier 
# Ransac_n --> randomly of points that are sampled to estimate a plane
# num_iterations - How often plane is sampled and verified

[a,b,c,d] = plane_model
print(f"Plane Equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1, 0, 0])
    
outlier_cloud = pcd.select_by_index(inliers, invert=True)
outlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])

print("Opening window... (Red = Detected Floor)")
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                      window_name="RANSAC Result",
                                      width=1024, height=768)

# https://www.open3d.org/docs/latest/tutorial/Basic/pointcloud.html

