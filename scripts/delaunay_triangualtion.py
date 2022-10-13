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

def DT_3d(pcdfile):
    """
    在3d点云上 进行 Delaunay Triangulation
    pcdfile: 点云txt 文件路径
    """
    
    print('[DT_3d] load point cloud from {}'.format(pcdfile))
    
    pcdarr = np.loadtxt(pcdfile)
    npt = pcdarr.shape[0]
    print('pcd size: ', pcdarr.shape)
    # 输出 xyz 的区域  目前显然有尺度问题
    # print('[raw pcd] x y z min:\n', pcdarr.min(axis=0))
    # print('[raw pcd] x y z max:\n', pcdarr.max(axis=0))
    
    # copy from point-nerf data/data_utils.py
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcdarr[:, :3]) # 相机位置的点云
    # pcd.normals = o3d.utility.Vector3dVector(pcdarr[:, :3] / np.linalg.norm(pcdarr[:, :3], axis=-1, keepdims=True)) # 每个位置点到原点的距离  每个位置单位向量
    # o3d.visualization.draw_geometries([pcd]) #
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.2, origin=[0, 0, 0]) #显示坐标系 1.0 20.0
    distances = pcd.compute_nearest_neighbor_distance() # np.asarray() (100, ) http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.compute_nearest_neighbor_distance
    avg_dist = np.mean(distances) # 平均每点到其最近邻的位置 0.425 ship data
    print('avg nn dist: {}'.format(avg_dist))
    radius = 3 * avg_dist # 从点云到mesh
    
    # 对点云滤波 open3d的函数
    # http://www.open3d.org/docs/release/tutorial/geometry/pointcloud_outlier_removal.html
    print("Statistical oulier removal")
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.8) # 20 2
    inlier_cloud = display_inlier_outlier(pcd, ind, vis=False)
    distances = inlier_cloud.compute_nearest_neighbor_distance() 
    avg_dist = np.mean(distances)
    radius = 3 * avg_dist
    pcdarr = np.asarray(inlier_cloud.points)
    inlier_cloud = o3d.geometry.PointCloud()
    inlier_cloud.points = o3d.utility.Vector3dVector(pcdarr[:, :3])
    # inlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=20))
    inlier_cloud.normals = o3d.utility.Vector3dVector(pcdarr[:, :3] / np.linalg.norm(pcdarr[:, :3], axis=-1, keepdims=True))
    
    # 保存次点云
    o3d.io.write_point_cloud('inlierpcd.ply', inlier_cloud)
    
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
    
    dec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(inlier_cloud, 0.08) # 此算法不需要normal 0.03 0.06
    
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

    vis_lst = [dec_mesh, inlier_cloud, mesh_frame]
    # vis_lst = [inlier_cloud, mesh_frame]
    # o3d.visualization.draw_geometries(vis_lst)
    
    o3d.io.write_triangle_mesh('triangle_pcd.ply', dec_mesh)
    
    # # 先画出点云 https://blog.csdn.net/qq_27197395/article/details/79414408
    # x = [pt[0] for pt in pcdarr]
    # y = [pt[1] for pt in pcdarr]
    # z = [pt[2] for pt in pcdarr]
    
    # fig=plt.figure(dpi=120)
    # ax1=fig.add_subplot(111,projection='3d')
    # #标题
    # plt.title('point cloud')
    # #利用xyz的值，生成每个点的相应坐标（x,y,z）
    # ax1.scatter(x,y,z,c='b',marker='.',s=2,linewidth=0,alpha=1,cmap='spectral')
    # ax1.axis('auto')          
    # ax1.set_xlabel('X Label')
    # ax1.set_ylabel('Y Label')
    # ax1.set_zlabel('Z Label')
    
    # tri = Delaunay(pcdarr)
    
    # # 画图参考 https://www.scaler.com/topics/matplotlib-triangulation/
    # # https://www.geeksforgeeks.org/tri-surface-plot-in-python-using-matplotlib/
    # # https://matplotlib.org/stable/gallery/mplot3d/trisurf3d_2.html
    # # matplotlib triplot 3d
    # plt.rcParams["figure.figsize"]=(10,10)
    # ax = plt.figure().gca(projection='3d')

    # ax.plot_trisurf(
    # pcdarr[:,0], pcdarr[:,1], pcdarr[:,2],
    # triangles=tri.simplices#, cmap=cm.viridis
    # )
    
    # plt.show()










if __name__ == '__main__':
    # parser command lines
    parser = argparse.ArgumentParser(description='''
      
    ''') 
    parser.add_argument('pcdfile', type=str, help='3d 点文件路径',default="office0_orb_mappts.txt") #
    # parser.add_argument('seq', help='所用序列',default='08')

    args = parser.parse_args()
    # 读取参数
    pcdfile = args.pcdfile
    # seq = args.seq
    # print("\n report result...\n")
    DT_3d(pcdfile)