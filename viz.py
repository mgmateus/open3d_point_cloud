import open3d as o3d

ABS_PATH = '/home/airsim/AirSim/ros/src/mxso_curl/mxso_curl/files/point_clouds/'

pcd_name = ABS_PATH + 'cloud_big.ply'

pcd = o3d.io.read_point_cloud(pcd_name)
o3d.visualization.draw_geometries([pcd])