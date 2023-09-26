#3D point clud segmentation based of normals and distances

import cv2
import numpy as np
import open3d as o3d
import matplotlib.colors as colors
import matplotlib.cm as cmx
import matplotlib.pyplot as plt

from open3d import *
def get_vector_angle(u,v):
    #print(u,v)
    uv=np.dot(u,v)
    #mu=np.linalg.norm(u)
    #mv=np.linalg.norm(v)
    ang = np.arccos(uv)
    ang = ang*180/np.pi
    if (ang > 90):
        ang = 180 - ang
    return (ang)
def get_plane_distance(n,p):
    d=np.dot(n,p)
    return (d)

def get_distance_group(dist,group1,distance_resolution):
    group={}
    group_center={}
    group[0]=[group1[0]]
    group_center[0]=dist[0]
    u=dist[0]
    
    for i in np.arange(1,len(group1)):
        v=dist[i]
        for y in group_center: #Add the new member in the matching group
            u=group_center[y]
            if abs(u-v) <=distance_resolution:
                group[y] +=[group1[i]]
                break
        else: #Create a new group center
            new_group=len(group_center)
            group_center[new_group]=v
            group[new_group]=[group1[i]]
    return (group, group_center)
    

if __name__ == "__main__":
    angle_resolution=10
    distance_resolution=0.05
    thresold=150
    print("Load a ply point cloud and render it")
    #pcd = o3d.io.read_point_cloud("c:/ply/test.ply")
    
    #pcd = o3d.io.read_point_cloud("../../TestData/fragment.ply")

    # color_raw = read_image("C:/Users/DELL/Python37/Rachna/data/rgb_2.jpg")
    # depth_raw = read_image("C:/Users/DELL/Python37/Rachna/data/depth_2.png")
    # #color_raw = read_image("C:/Users/DELL/Python37/Open3D-master/examples/TestData/RGBD/other_formats/TUM_color.png")
    # #depth_raw = read_image("C:/Users/DELL/Python37/Open3D-master/examples/TestData/RGBD/other_formats/TUM_depth.png")
    
    # rgbd_image = o3d.create_rgbd_image_from_tum_format(color_raw, depth_raw);
    
   
    # pcd = o3d.create_point_cloud_from_rgbd_image(rgbd_image, o3d.PinholeCameraIntrinsic(
    #         o3d.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    #Flip it, otherwise the pointcloud will be upside down
    
    #load point cloud
    filename="./Fig4g.xyz"
    pcd_temp=np.loadtxt(filename 
                #    ,delimiter=','
                )
    pcd=o3d.geometry.PointCloud()
    pcd.points=o3d.utility.Vector3dVector(pcd_temp[:,:3]) ###这里直接拿边框就行了，少delta应该问题不大
    pcd.paint_uniform_color(color=[0, 0, 1])

    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        
    #o3d.io.write_point_cloud("C:/Users/DELL/Python37/Rachna/rgb_3.pcd", pcd)
    #o3d.io.write_point_cloud("C:/Users/DELL/Python37/Rachna/test.ply", pcd)
    if not pcd.has_points():
        print("Unable to propely load point cloud. check the file name and location")
        exit()
    print(pcd)
    o3d.visualization.draw_geometries([pcd])
    downpcd = pcd.voxel_down_sample(voxel_size=0.005)
    # downpcd = o3d.geometry.voxel_down_sample(pcd, voxel_size=0.005)
    cl, ind = downpcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    # cl, ind = o3d.geometry.statistical_outlier_removal(downpcd,nb_neighbors=20, std_ratio=2.0)
    
    downpcd = downpcd.select_by_index(ind)##这里好像是发生了一些变化了
    # downpcd = o3d.geometry.select_down_sample(downpcd, ind)
    downpcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))
    # o3d.geometry.estimate_normals(downpcd,search_param=o3d.geometry.KDTreeSearchParamHybrid(
    #     radius=0.1, max_nn=30))
    #o3d.visualization.draw_geometries([downpcd])
    
    #print(downpcd)
        
    #downpcd=pcd
    #o3d.visualization.draw_geometries([downpcd])

    normals= np.asarray(downpcd.normals)
    group={}
    group_center={}
    group[0]=[0]
    group_center[0]=normals[0]
    u=normals[0]
    for i in np.arange(1,len(normals)):
        v=normals[i]
        for y in group_center: #Add the new member in the matching group
            u=group_center[y]
            if get_vector_angle(u,v) <=angle_resolution:
                group[y] +=[i]
                break
        else: #Create a new group center
            new_group=len(group_center)
            group_center[new_group]=v
            group[new_group]=[i]



    planes={}
    no_of_planes=0
    for i in group_center:        
        n=group_center[i]
        dist=[]
        for idx in group[i]:
            p= downpcd.points[idx]
            dist+=[get_plane_distance(n,p)]
##        plt.plot(np.sort(dist))
##        plt.show()
        a,b=get_distance_group(dist,group[i],distance_resolution)
        planes[i]=[a]
        no_of_planes+=len(b)

    jet=plt.get_cmap('jet')
    cNorm = colors.Normalize(vmin=0,vmax=10*no_of_planes)
    scalarMap=cmx.ScalarMappable(norm=cNorm,cmap=jet)

    current_plane_no=0
    no_large_planes=0
    for i in planes:
        p=planes[i]
        for idx in p[0]:
            a=p[0][idx]
            if len(a) <thresold:
                np.asarray(downpcd.colors)[a[:],:]=[1,1,1]
                current_plane_no+=1
                continue
            plane_color=np.random.randint(0,10*no_of_planes)
            print(plane_color)
            colorValue=scalarMap.to_rgba(plane_color)
            np.asarray(downpcd.colors)[a[:],:]=list(colorValue[0:3])
            current_plane_no+=1
            no_large_planes+=1
            
    o3d.visualization.draw_geometries([downpcd])
    
        

