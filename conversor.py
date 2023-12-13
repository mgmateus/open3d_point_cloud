#!/usr/bin/env python3


import os
import argparse
import sys

import open3d as o3d


class Conversor:
    
    pcds = {}
    
    @staticmethod
    def save_pcd(pcd, pcd_path):
        o3d.io.write_point_cloud(pcd_path, pcd)
        return True
    
    @staticmethod
    def mesh2pcd(mesh_path, number_of_points= 200000, visualize= False):
        mesh = o3d.io.read_triangle_mesh(mesh_path)

        mesh.compute_vertex_normals()
        draw_mesh = [mesh]
        
        pcd = mesh.sample_points_poisson_disk(number_of_points=number_of_points, init_factor=5)
        draw_pcd = [pcd]
        
        
        if visualize:
            o3d.visualization.draw_geometries(draw_mesh)
            o3d.visualization.draw_geometries(draw_pcd)
        
        return pcd
    
    @classmethod
    def dir_mesh2pcd(cls, mesh_dir, number_of_points= 200000, visualize= False):
        for _, _, files in os.walk(mesh_dir):
            for mesh in files:
                print(mesh)
                mesh_path = mesh_dir + "/" + mesh
                name = mesh.split(".")[0] + ".pcd"
                
                pcd = cls.mesh2pcd(mesh_path, number_of_points, visualize)
                
                    
                cls.pcds[name] = pcd
        return cls.pcds
        
        
        
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Configuration')
    parser.add_argument('--action', type= str, choices=['mesh2pcd', 'pcd2mesh'], help='')
    parser.add_argument('--file_or_dir', type= str, required=True, choices=['file', 'dir'], help='')
    parser.add_argument('--mesh_path', type= str, required=False, help='')
    parser.add_argument('--point_cloud_path', type= str, required=False)
    parser.add_argument('--save', type= bool, required=False, help='')
    parser.add_argument('--number_of_points', type= int, required=False, help='')
    parser.add_argument('--visualize', type= bool, required=False, help='')
    
    args = parser.parse_args()
    
    action = args.action
    fod = args.file_or_dir
    mesh_path = args.mesh_path
    pcd_path = args.point_cloud_path
    save = args.save
    nop = args.number_of_points
    visualize = args.visualize
    
    if action == "mesh2pcd":
        if fod == "file":
            if nop:
                pcd = Conversor.mesh2pcd(mesh_path=mesh_path, number_of_points=nop, \
                                    visualize=visualize)
            elif nop and visualize:
                pcd = Conversor.mesh2pcd(mesh_path=mesh_path,number_of_points=nop, \
                    visualize=visualize)
                
            elif visualize:
                pcd = Conversor.mesh2pcd(mesh_path=mesh_path, visualize=visualize)
            else:
                pcd = Conversor.mesh2pcd(mesh_path=mesh_path)
                
            if save:
                if pcd_path:
                    Conversor.save_pcd(pcd=pcd, pcd_path=pcd_path)
                else:
                    print("Path with name to point cloud save is None!")
                    sys.exit()
                    
        else:
            pcds = Conversor.dir_mesh2pcd(mesh_dir=mesh_path)
            
            if save:
                if pcd_path:
                    for name, pcd in pcds.items():
                        pcd_path += "/" + name
                        Conversor.save_pcd(pcd=pcd, pcd_path=pcd_path)
                else:
                    print("Path to point cloud save is None!")
                    sys.exit()
                
                    
        