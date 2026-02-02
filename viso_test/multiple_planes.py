import open3d as o3d
import numpy as np
import random
import os

def find_walls_force(filename):
    if not os.path.exists(filename):
        print(f"Error: {filename} not found.")
        return

    print(f"Loading {filename}...")
    pcd = o3d.io.read_point_cloud(filename)
    
    # Downsample (5cm)
    # print("Downsampling (5cm)...")
    # pcd = pcd.voxel_down_sample(voxel_size=0.05)
    
    # Remove noise 
    # print("Removing noise...")
    # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
    # pcd = pcd.select_by_index(ind)

    remaining_cloud = pcd
    planes_found = []
    
    #Parameters for plane-fitting 
    threshold = 0.1
    min_pts = 500
    
    # Horizontal Limit: Only allow 2 horizontal planes (Floor + Ceiling)
    # Once we find 2, we ignore all other horizontal noise layers.
    horizontal_count = 0
    max_horizontal = 0

    print("Running Iterative RANSAC...")

    while True:
        # standard RANSAC
        plane_model, inliers = remaining_cloud.segment_plane(
            distance_threshold=threshold,
            ransac_n=3,
            num_iterations=2000
        )
        
        if len(inliers) < min_pts:
            break

        # Check Orientation
        [a, b, c, d] = plane_model
        
        # Check if a plane is horizontal (Floor/Ceiling)
        is_horizontal = abs(c) > 0.5 

        keep_plane = True
        
        if is_horizontal:
            if horizontal_count < max_horizontal:
                horizontal_count += 1
                type_name = "FLOOR/CEILING"
            else:
                keep_plane = False
                type_name = "Useless (Discarded)"
        else:
            type_name = "WALL"

        if keep_plane:
            print(f" Found {type_name}: {len(inliers)} points")
            
            plane_cloud = remaining_cloud.select_by_index(inliers)
            r = random.random()
            g = random.random()
            b = random.random()
            plane_cloud.paint_uniform_color([r, g, b])
            
            planes_found.append(plane_cloud)
        else:
            print(f" Removing {type_name}: {len(inliers)} points")

        remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)
        
        if len(remaining_cloud.points) < min_pts:
            break
        
        if len(planes_found) > 10:
            break

    # Visualize
    print(f"Found {len(planes_found)} valid planes.")
    
    # Paint leftovers Grey
    remaining_cloud.paint_uniform_color([0.2, 0.2, 0.2]) 
    planes_found.append(remaining_cloud)
    
    o3d.visualization.draw_geometries(planes_found, 
                                      window_name="Floor Result",
                                      width=1024, height=768)

if __name__ == "__main__":
    find_walls_force("cloud_pillar.ply")
    
# import open3d as o3d
# import numpy as np
# import os

# def MultiplePlanes(points):



#     plane_list = []
#     N = len(points) 

#     print("Running RANSAC")
#     plane_model, inliers = pcd.segment_plane(distance_threshold = 0.05, 
#                                             ransac_n = 3, 
#                                             num_iterations = 1000)

#     # Distance Threshold - Max Distance from Plane to be inlier 
#     # Ransac_n --> randomly of points that are sampled to estimate a plane
#     # num_iterations - How often plane is sampled and verified

#     [a,b,c,d] = plane_model
#     print(f"Plane Equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

#     inlier_cloud = pcd.select_by_index(inliers)
#     inlier_cloud.paint_uniform_color([1, 0, 0])
        
#     outlier_cloud = pcd.select_by_index(inliers, invert=True)
#     outlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])

#     print("Opening window... (Red = Detected Floor)")
#     o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
#                                         window_name="RANSAC Result",
#                                         width=1024, height=768)

# # https://www.open3d.org/docs/latest/tutorial/Basic/pointcloud.html
# # https://github.com/yuecideng/Multiple_Planes_Detection/blob/master/utils.py

# if __name__ == "__main__": 
#     import random 
#     filename = "cloud_pillar.ply"
#     pcd = o3d.io.read_point_cloud(filename)

#     pcd = o3d.geometry.PointCloud() 
#     xyz = np.ndarray() 
#     pcd.points = o3d.utiliy.Vector3DVector(xyz)

#     np.asarray(pcd.points)
#     from utils import *


# def DetectMultiPlanes(points, min_ratio=0.05, threshold=0.01, iterations=1000):
#     """ Detect multiple planes from given point clouds

#     Args:
#         points (np.ndarray): 
#         min_ratio (float, optional): The minimum left points ratio to end the Detection. Defaults to 0.05.
#         threshold (float, optional): RANSAC threshold in (m). Defaults to 0.01.

#     Returns:
#         [List[tuple(np.ndarray, List)]]: Plane equation and plane point index
#     """

#     plane_list = []
#     N = len(points)
#     target = points.copy()
#     count = 0

#     while count < (1 - min_ratio) * N:
#         w, index = PlaneRegression(
#             target, threshold=threshold, init_n=3, iter=iterations)
    
#         count += len(index)
#         plane_list.append((w, target[index]))
#         target = np.delete(target, index, axis=0)

#     return plane_list


# if __name__ == "__main__":
#     import random
#     import time
#     points = RemoveNoiseStatistical(points, nb_neighbors=50, std_ratio=0.5)

#     t0 = time.time()
#     results = DetectMultiPlanes(points, min_ratio=0.05, threshold=0.005, iterations=2000)
#     print('Time:', time.time() - t0)
#     planes = []
#     colors = []
#     for _, plane in results:

#         r = random.random()
#         g = random.random()
#         b = random.random()

#         color = np.zeros((plane.shape[0], plane.shape[1]))
#         color[:, 0] = r
#         color[:, 1] = g
#         color[:, 2] = b

#         planes.append(plane)
#         colors.append(color)
    
#     planes = np.concatenate(planes, axis=0)
#     colors = np.concatenate(colors, axis=0)

#     # Draw Result 
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(points)
#     pcd.colors = o3d.utility.Vector3dVector(colors)
#     o3d.visualization.draw_geometries([pcd])


# def ReadPlyPoint(fname):
#     pcd = o3d.io.read_point_cloud(fname)

#     return PCDToNumpy(pcd)


# def NumpyToPCD(xyz):
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(xyz)

#     return pcd


# def PCDToNumpy(pcd):
#     return np.asarray(pcd.points)


# def RemoveNan(points):
#     return points[~np.isnan(points[:, 0])]


# def RemoveNoiseStatistical(pc, nb_neighbors=20, std_ratio=2.0):
#     pcd = NumpyToPCD(pc)
#     cl, ind = pcd.remove_statistical_outlier(
#         nb_neighbors=nb_neighbors, std_ratio=std_ratio)

#     return PCDToNumpy(cl)


# def DownSample(pts, voxel_size=0.003):
#     p = NumpyToPCD(pts).voxel_down_sample(voxel_size=voxel_size)

#     return PCDToNumpy(p)


# def PlaneRegression(points, threshold=0.01, init_n=3, iter=1000):
#     pcd = NumpyToPCD(points)

#     w, index = pcd.segment_plane(
#         threshold, init_n, iter)

#     return w, index