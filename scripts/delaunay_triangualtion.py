"""
对orb-slam2 保存的点云 进行 德劳内三角剖分
可视化
并保存，后续作为 nice-slam 的coarse geometry 先验
2022/10/12
https://zhuanlan.zhihu.com/p/459884570
https://blog.csdn.net/weixin_40524689/article/details/123235003
https://stackoverflow.com/questions/56686838/point-cloud-triangulation-using-marching-cubes-in-python-3
https://stackoverflow.com/questions/6537657/python-scipy-delaunay-plotting-point-cloud
https://stackoverflow.com/questions/24163252/python-section-of-a-tetrahedralized-scipy-delaunay-3d-cloud-of-points
https://stackoverflow.com/questions/20025784/how-to-visualize-3d-delaunay-triangulation-in-python/53284599#53284599
"""

import sys, os, argparse, glob, copy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy.spatial import Delaunay
import open3d as o3d
import pyvista as pv
from align_scale import load_traj


def display_inlier_outlier(cloud, ind, vis=True):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    # 统计内外点数目
    nin = np.asarray(inlier_cloud.points).shape[0]
    nout = np.asarray(outlier_cloud.points).shape[0]
    print('inlier: {}, outlier: {}'.format(nin, nout))
    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    if vis:
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
                                        #   ,
                                        #   zoom=0.3412,
                                        #   front=[0.4257, -0.2125, -0.8795],
                                        #   lookat=[2.6172, 2.0475, 1.532],
                                        #   up=[-0.0694, -0.9768, 0.2024])
    return inlier_cloud


def get_align_transformation(est_pcd, gt_pcd,  tr_init = None):
    """
    Get the transformation matrix to align the reconstructed mesh to the ground truth mesh.
    """
    trans_init = np.eye(4)
    if tr_init is not None:
        trans_init = tr_init
    threshold = 0.05
    # target 点云 计算法线
    gt_pcd.estimate_normals()
    reg_p2p = o3d.pipelines.registration.registration_icp(
        est_pcd, gt_pcd, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()) # TransformationEstimationPointToPlane
    transformation = reg_p2p.transformation
    return transformation
    


def DT_3d(rawpcdfile, alignpcdfile):
    """
    在3d点云上 进行 Delaunay Triangulation
    rawpcdfile: 点云txt 文件路径
    alignpcdfile: 尺度变换后的 点云文件
    """
    
    print('[DT_3d] load raw point cloud from {}'.format(rawpcdfile))
    
    pcdarr = np.loadtxt(rawpcdfile)
    npt = pcdarr.shape[0]
    print('pcd size: ', pcdarr.shape)
    # 输出 xyz 的区域  目前显然有尺度问题
    # print('[raw pcd] x y z min:\n', pcdarr.min(axis=0))
    # print('[raw pcd] x y z max:\n', pcdarr.max(axis=0))
    
    print('[DT_3d] load aligned point cloud from {}'.format(alignpcdfile))
    alignpcdarr = np.loadtxt(alignpcdfile)
    if npt != alignpcdarr.shape[0]:
        print('ERROR: wrong shape! exit')
        return -1
    # copy from point-nerf data/data_utils.py
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcdarr[:, :3]) # 相机位置的点云
    # pcd.normals = o3d.utility.Vector3dVector(pcdarr[:, :3] / np.linalg.norm(pcdarr[:, :3], axis=-1, keepdims=True)) # 每个位置点到原点的距离  每个位置单位向量
    # o3d.visualization.draw_geometries([pcd]) #
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=1.0, origin=[0, 0, 0]) #显示坐标系 1.0 20.0
    distances = pcd.compute_nearest_neighbor_distance() # np.asarray() (100, ) http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.compute_nearest_neighbor_distance
    avg_dist = np.mean(distances) # 平均每点到其最近邻的位置 0.425 ship data
    print('avg nn dist: {}'.format(avg_dist))
    radius = 3 * avg_dist # 从点云到mesh
    
    alignpcd = o3d.geometry.PointCloud()
    alignpcd.points = o3d.utility.Vector3dVector(alignpcdarr[:, :3]) # 相机位置的点云
    alignpcd.normals = o3d.utility.Vector3dVector(alignpcdarr[:, :3] / np.linalg.norm(alignpcdarr[:, :3], axis=-1, keepdims=True)) # 每个位置点到原点的距离  每个位置单位向量
    distances = alignpcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances) # 平均每点到其最近邻的位置 0.425 ship data
    print('align avg nn dist: {}'.format(avg_dist))
    
    # _ , gtc2ws = load_traj("dataset/Replica/office0/traj.txt")
    # fristc2w = gtc2ws[0]
    # print('frist gt c2w: \n', fristc2w)
    # gtmesh = o3d.io.read_triangle_mesh('dataset/Replica/cull_replica_mesh/office0.ply')
    # print('raw gt mesh box: \n', gtmesh.get_axis_aligned_bounding_box())
    # o3d_gt_pc = o3d.geometry.PointCloud(points=gtmesh.vertices)
    # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.8) # 20 2
    # pcd1 = display_inlier_outlier(pcd, ind, vis=False)
    # tr = get_align_transformation(pcd1, o3d_gt_pc, tr_init = fristc2w)
    # print('align tr: \n', tr)
    # gtmesh_1 = copy.deepcopy(gtmesh).transform(np.linalg.inv(tr)) # np.linalg.inv(tr)
    # print('gt mesh1 box: \n', gtmesh_1.get_axis_aligned_bounding_box())
    # o3d.visualization.draw_geometries([mesh_frame, gtmesh_1, pcd1]) # gtmesh
    print('raw pcd box: \n', pcd.get_axis_aligned_bounding_box())
    print('aligned pcd box: \n', alignpcd.get_axis_aligned_bounding_box())
    # 先按真实bound 过滤一次
    pred_v = np.asarray(alignpcd.points)
    pcd_clip = pcd.select_by_index(np.where((pred_v[:, 0] < 4.1) & (pred_v[:, 0] > -2.2) & (pred_v[:, 1] < 3.7) & (pred_v[:, 1] > -2.9) & (pred_v[:, 2] < 3.9) & (pred_v[:, 2] > -2.4))[0])
    print('pcd_clip box: \n', pcd_clip.get_axis_aligned_bounding_box())
    alignpcd_clip = alignpcd.select_by_index(np.where((pred_v[:, 0] < 4.1) & (pred_v[:, 0] > -2.2) & (pred_v[:, 1] < 3.7) & (pred_v[:, 1] > -2.9) & (pred_v[:, 2] < 3.9) & (pred_v[:, 2] > -2.4))[0])
    print('aligned pcd_clip box: \n', alignpcd_clip.get_axis_aligned_bounding_box())
    
    # 对点云滤波 open3d的函数
    # http://www.open3d.org/docs/release/tutorial/geometry/pointcloud_outlier_removal.html
    print("Statistical oulier removal")
    cl, ind = alignpcd_clip.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.9) # 20 2
    aligninlier_cloud = display_inlier_outlier(alignpcd_clip, ind, vis=True)
    print('align inlier pcd box: \n', aligninlier_cloud.get_axis_aligned_bounding_box())
    inlier_cloud = display_inlier_outlier(pcd_clip, ind, vis=True)
    print('raw inlier pcd box: \n', inlier_cloud.get_axis_aligned_bounding_box())
    # 原始坐标系 用于和原yaml参数对比 my2
    # bound: [[-2.1,2.4],[-3.2,1.9],[-1.2,1.8]] # 
    # gt depth 投影后的点云 边界
    # min([-2.00679093, -3.14577566, -1.14950645])
    # max([2.39498164, 1.79798702, 1.77147197])
    # distances = inlier_cloud.compute_nearest_neighbor_distance() 
    # avg_dist = np.mean(distances)
    # radius = 3 * avg_dist
    pcdarr = np.asarray(inlier_cloud.points)
    inlier_cloud = o3d.geometry.PointCloud()
    inlier_cloud.points = o3d.utility.Vector3dVector(pcdarr[:, :3])
    # inlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=20))
    # inlier_cloud.normals = o3d.utility.Vector3dVector(pcdarr[:, :3] / np.linalg.norm(pcdarr[:, :3], axis=-1, keepdims=True))
    
    pcdarr = np.asarray(aligninlier_cloud.points)
    aligninlier_cloud = o3d.geometry.PointCloud()
    aligninlier_cloud.points = o3d.utility.Vector3dVector(pcdarr[:, :3])
    # inlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=20))
    aligninlier_cloud.normals = o3d.utility.Vector3dVector(pcdarr[:, :3] / np.linalg.norm(pcdarr[:, :3], axis=-1, keepdims=True))
    
    # 保存次点云
    o3d.io.write_point_cloud('inlierpcd.ply', inlier_cloud)
    o3d.io.write_point_cloud('align_inlierpcd.ply', aligninlier_cloud)
    if np.asarray(inlier_cloud.points).shape[0] != np.asarray(aligninlier_cloud.points).shape[0]:
        print('[ERROR] after process size wrong! exit')
        return -1
    
    # # https://docs.pyvista.org/api/core/_autosummary/pyvista.DataSetFilters.delaunay_3d.html#pyvista.DataSetFilters.delaunay_3d
    # pmesh = pv.read('inlierpcd.ply')
    # grid = pmesh.delaunay_3d(alpha=0.1, progress_bar=True)
    # edges = grid.extract_all_edges()
    # edges.plot(line_width=5, color='k', background='w')
    # https://docs.pyvista.org/api/core/_autosummary/pyvista.PolyDataFilters.reconstruct_surface.html#pyvista.PolyDataFilters.reconstruct_surface
    
    # surf = pmesh.reconstruct_surface(nbr_sz=30, progress_bar=True) 不行
    
    # pl = pv.Plotter()
    # pl.add_mesh(pmesh)
    # pl.add_mesh(grid, color=True, show_edges=True) # grid edges surf
    # pl.show()
    
    dec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(inlier_cloud, 0.09) 
    
    align_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(aligninlier_cloud, 0.18) # 此算法不需要normal 0.03 0.06 0.08 0.16 0.18 0.20
    
    # 三角剖分内的元素
    # np.asarray(dec_mesh.triangles) # 每个三角形 顶点组成 id 对应下面 的坐标值
    # np.asarray(dec_mesh.vertices)
    
    
    # inlier_cloud.estimate_normals( # 计算挺慢
    # search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=20))
    
    # dec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(inlier_cloud, o3d.utility.DoubleVector(
    #     [radius, radius * 2])) # 此算法需要normal
    
    # poisson 
    # dec_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        # inlier_cloud, depth=6) # 此算法也需要normal
    
    # dec_mesh = dec_mesh.simplify_quadric_decimation(100000)
    # dec_mesh.remove_degenerate_triangles()
    # dec_mesh.remove_duplicated_triangles()
    # dec_mesh.remove_duplicated_vertices()
    # dec_mesh.remove_non_manifold_edges()

    vis_lst = [dec_mesh, align_mesh, inlier_cloud, aligninlier_cloud, mesh_frame]
    # vis_lst = [inlier_cloud, mesh_frame]
    o3d.visualization.draw_geometries(vis_lst)
    
    o3d.io.write_triangle_mesh('triangle_pcd.ply', dec_mesh)
    o3d.io.write_triangle_mesh('align_triangle_pcd.ply', align_mesh)
    
    

if __name__ == '__main__':
    # parser command lines
    parser = argparse.ArgumentParser(description='''
      
    ''') 
    parser.add_argument('alignpcdfile', type=str, help='3d 点文件路径',default="office0_orbalign_mappts.txt") # office0_orbalign_mappts.txt office0_orb_mappts.txt
    parser.add_argument('rawpcdfile', type=str, help='3d 点文件路径',default="office0_orb_mappts.txt")
    # parser.add_argument('seq', help='所用序列',default='08')

    args = parser.parse_args()
    # 读取参数
    rawpcdfile = args.rawpcdfile
    alignpcdfile = args.alignpcdfile
    # seq = args.seq
    # print("\n report result...\n")
    DT_3d(rawpcdfile, alignpcdfile)