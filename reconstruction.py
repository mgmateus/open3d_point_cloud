import rospy

import numpy as np
import open3d as o3d


from airsim_ros_pkgs.msg import Image
from std_msgs.msg import String

from airsim_base.types import Pose
from airsim_gym.airsim_simulation_resources.utils import image_transport

class R3D:
    def __init__(self, 
                 img_width : int, /
                 img_height : int, /
                 img_fov : float, /
                 rgb_topic : str, /
                 seg_topic : str, /
                 depth_topic : str) -> None:
        
        rospy.Subscriber(rgb_topic, Image, self._callback_rgb)
        rospy.Subscriber(seg_topic, Image, self._callback_seg)
        rospy.Subscriber(depth_topic, Image, self._callback_depth)
        self._pub_info = rospy.Publisher("Reconstruction_info", \
                                          String, queue_size=10)
        
        self._rgb = None
        self._seg = None
        self._depth = None
        self._map = o3d.PointCloud()
        self._trajectory = []
        
        self.fov_rad = img_fov * np.pi/180
        self.fd = (img_width/2.0) / np.tan(self.fov_rad/2.0)
        
        self._camera = o3d.camera.PinholeCameraIntrinsic()
        self._camera.set_intrinsics(img_width, img_height, self.fd, self.fd, img_width/2 - 0.5, img_height/2 - 0.5)
        
        
    def callback_image(func):
        def callback(self, *args, **kwargs):
            data, img_type = func(self, *args, **kwargs)
            if data:
                cv_img = image_transport(data)
                self.__setattr__("_"+img_type, cv_img)
            else:
                info = f"Error in {img_type} cam!"
                self._pub_info.publish(info)

        return callback
    
    @callback_image
    def _callback_rgb(self, data):
        return data, "rgb"
    
    @callback_image
    def _callback_seg(self, data):
        return data, "seg"
    
    @callback_image
    def _callback_depth(self, data):
        return data, "depth"
    
    def _get_transform_matrix(self, vehicle_pose : Pose) -> np.array:
        x, y, z = vehicle_pose.position
        qx, qy, qz, qw = vehicle_pose.orientation
        
        T = np.eye(4)
        T[:3, :3] = [-y, -z, -x]
        
        R = np.eye(4)
        R[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion((qw, qy, qz, qx))
        
        C = np.array([
            [1., 0., 0., 0.,],
            [0, 0., -1., 0.,],
            [0, 1., 0., 0.,],
            [0., 0., 0., 1.,]
        ])
        
        return R.T @ T @ C
    
    def create_map(self, vehicle_pose : Pose, type_vis : str, depth_trunc : float, viz : bool = True):
        color_map = self.__getattribute__("_" + type_vis)
        color_img= o3d.geometry.Image(np.asarray(color_map))
        depth_img = o3d.geometry.Image(self._depth)
        F = self._get_transform_matrix(vehicle_pose)
        
        rgbd_img = o3d.geometry.RGBDImage.create_from_color_and_depth(color_img, depth_img, depth_scale=1.0, depth_trunc=depth_trunc, convert_rgb_to_intensity=False)
        rgbd_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_img, self._camera, extrinsic = F)
        
        self._map += rgbd_pcd
        self._trajectory.append(o3d.LineSet.create_camera_visualization(self._camera, F))
        
        if viz:
            poses = [self._map]
            poses.extend(self._trajectory)
            o3d.visualization.draw_geometries(poses)
            
        
        
        
        
    