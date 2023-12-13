import open3d as o3d

ABS_PATH = '/home/hydrone/mm_lib/ROS/airsim_ws/mxso_curl/mxso_curl/files/point_clouds/'

file = ABS_PATH + 'platform.pcd'

pcd = o3d.io.read_point_cloud(file)
axes = o3d.geometry.TriangleMesh.create_coordinate_frame()

plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=10000)
[a, b, c, d] = plane_model
plane_pcd = pcd.select_by_index(inliers)
plane_pcd.paint_uniform_color([1.0, 0, 0])
stockpile_pcd = pcd.select_by_index(inliers, invert=True)
stockpile_pcd.paint_uniform_color([0, 0, 1.0])
o3d.visualization.draw_geometries([plane_pcd, stockpile_pcd, axes])