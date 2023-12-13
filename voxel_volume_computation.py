from functools import reduce
import os
import math
import cv2

import open3d as o3d
import numpy as np

from scipy.spatial import Delaunay

ABS_PATH = '/home/hydrone/mm_lib/ROS/airsim_ws/mxso_curl/mxso_curl/files/point_clouds/'

file1 = ABS_PATH + 'plataform.PLY'
file2 = ABS_PATH + 'platform.pcd'
teste = ABS_PATH + 'cloud.ply'
teste2 = ABS_PATH + 'cloud2.ply'

cw =  ABS_PATH + 'cloud_underw.ply'
cw2 = ABS_PATH + 'cloud_w.ply'

w = ABS_PATH + 'cloud2.ply'

def load_cloud(file_path):
    """
    Carrega nuvem de pontos e salva resultado em matriz numpy.
    
    Parâmetros
    ----------
    file_path: Caminho completo para o arquivo com a nuvem
    
    Retornos
    --------
    xyz: Matriz (n x 3) com os pontos da nuvem
    
    """
    
    pcd = o3d.io.read_point_cloud(file_path)
    xyz = np.asarray(pcd.points)
    
    return xyz

def calculate_voxel_dif(v1, v2):
    print(v1.get_voxels())
    src = np.asarray(v1.get_voxels())
    tgt = np.asarray(v2.get_voxels())
    diff = np.abs(src.T - tgt)

    print(diff)
pcd1 = o3d.io.read_point_cloud(w)
pcd2 = o3d.geometry.PointCloud()

points = load_cloud(w)
pcd2.points = o3d.utility.Vector3dVector(points[:12000, :])

vg1 = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd1, voxel_size=2.5)
vg2 = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd2, voxel_size=2.5)

v = vg1.get_voxels()

va = np.asarray(v)
fv = len(va)
v = (fv*vg1.voxel_size**3)

print(v*1e-3)

d = 100
dpx = 50
dpy = 50
VT = v
vt0 = 10
vtf = 20
D = -np.sqrt(dpx**2 + dpy**2) - VT + (vtf - vt0) + d
print(D)
#o3d.visualization.draw_geometries([vg1, vg2])


def calcEntropy(img):
    entropy = []

    hist = cv2.calcHist([img], [0], None, [256], [0, 255])
    total_pixel = img.shape[0] * img.shape[1]

    for item in hist:
        probability = item / total_pixel
        if probability == 0:
            en = 0
        else:
            en = -1 * probability * (np.log(probability) / np.log(2))
        entropy.append(en)

    sum_en = np.sum(entropy)
    return sum_en

