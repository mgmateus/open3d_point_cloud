import open3d as o3d
import numpy as np
import copy
# Initialize functions
def draw_registration_result(source, target, transformation):
    """
    param: source - source point cloud
    param: target - target point cloud
    param: transformation - 4 X 4 homogeneous transformation matrix
    """
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp], zoom=0.4459, front=[0.9288, -0.2951, -0.2242], lookat=[1.6784, 2.0612, 1.4451], up=[-0.3402, -0.9189, -0.1996])
    
def find_nearest_neighbors(source_pc, target_pc, nearest_neigh_num):
    # Find the closest neighbor for each anchor point through KDTree
    point_cloud_tree = o3d.geometry.KDTreeFlann(source_pc)
    # Find nearest target_point neighbor index
    points_arr = []
    for point in target_pc.points:
        [_, idx, _] = point_cloud_tree.search_knn_vector_3d(point, nearest_neigh_num)
        points_arr.append(source_pc.points[idx[0]])
    return np.asarray(points_arr)


def icp(source, target):
    source.paint_uniform_color([0.5, 0.5, 0.5])
    target.paint_uniform_color([0, 0, 1])
    #source_points = np.asarray(source.points) # source_points is len()=198835x3 <--> 198835 points that have (x,y,z) val
    target_points = np.asarray(target.points)
    # Since there are more source_points than there are target_points, we know there is not
    # a perfect one-to-one correspondence match. Sometimes, many points will match to one point,
    # and other times, some points may not match at all.

    transform_matrix = np.asarray([[0.862, 0.011, -0.507, 0.5], [-0.139, 0.967, -0.215, 0.7], [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    source = source.transform(transform_matrix)
    draw_registration_result(source, target, transform_matrix)
    # While loop variables
    curr_iteration = 0
    cost_change_threshold = 0.0000000001
    curr_cost = 1000000
    prev_cost = 10000000

    while (True):
        # 1. Find nearest neighbors
        new_source_points = find_nearest_neighbors(source, target, 1)

        # 2. Find point cloud centroids and their repositions
        source_centroid = np.mean(new_source_points, axis=0)
        target_centroid = np.mean(target_points, axis=0)
        source_repos = np.zeros_like(new_source_points)
        target_repos = np.zeros_like(target_points)
        source_repos = np.asarray([new_source_points[ind] - source_centroid for ind in range(len(new_source_points))])
        target_repos = np.asarray([target_points[ind] - target_centroid for ind in range(len(target_points))])

        # 3. Find correspondence between source and target point clouds
        cov_mat = target_repos.transpose() @ source_repos

        U, X, Vt = np.linalg.svd(cov_mat)
        R = U @ Vt
        t = target_centroid - R @ source_centroid
        t = np.reshape(t, (1,3))
        curr_cost = np.linalg.norm(target_repos - (R @ source_repos.T).T)
        print("Curr_cost=", curr_cost)
        if ((prev_cost - curr_cost) > cost_change_threshold):
            prev_cost = curr_cost
            transform_matrix = np.hstack((R, t.T))
            transform_matrix = np.vstack((transform_matrix, np.array([0, 0, 0, 1])))
            # If cost_change is acceptable, update source with new transformation matrix
            source = source.transform(transform_matrix)
            curr_iteration += 1
        else:
            break
    print("\nIteration=", curr_iteration)
    # Visualize final iteration and print out final variables
    draw_registration_result(source, target, transform_matrix)
    return source, target, transform_matrix


pcd1 = o3d.io.read_point_cloud('/home/airsim/AirSim/ros/src/dmcurl_nbv/point_clouds/plataform.pcd')
pcd2 = o3d.io.read_point_cloud('/home/airsim/AirSim/ros/src/dmcurl_nbv/point_clouds/platform.pcd')

#t = icp(pcd1, pcd2)
points = np.asarray(pcd2.points)[:10000,:]
pcd2.points = o3d.utility.Vector3dVector(points)


pcd1_, pcd2_, _ = icp(pcd1, pcd2)
#octree1, octree2 = o3d.geometry.Octree(max_depth=4), o3d.geometry.Octree(max_depth=4)
#octree1.convert_from_point_cloud(pcd1_, size_expand=0.01)
#octree2.convert_from_point_cloud(pcd2_, size_expand=0.01)
#o3d.visualization.draw_geometries([octree1, octree2])
#voxel_grid1 = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd1_, voxel_size=0.05)
#voxel_grid2 = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd2_, voxel_size=0.05)
#o3d.visualization.draw_geometries([voxel_grid1, voxel_grid2])
print(voxel_grid1.get_voxels(), voxel_grid2.get_voxels())