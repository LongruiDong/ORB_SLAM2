"""
对orb-slam2 保存的点云  
根据估计位姿和gt 之间的尺度对齐结果 来 scale 得到的单目下 绝对尺度不对的点云
2022/10/13

"""

from ast import Not
import sys, os, argparse, glob, copy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import open3d as o3d
from mathutils import Matrix

fps = 10 # 生成伪时间 但要根据设置的帧率

def align(model,data,calc_scale=True):
    """Align two trajectories using the method of Horn (closed-form).
    
    Input:
    model -- first trajectory (3xn) 应该得是gt吧 这是为了 最后评估时的尺度是在gt上 否则结果数字失真
    data -- second trajectory (3xn) est
    
    Output:
    rot -- rotation matrix (3x3)  此SE3变换应该是把 model 变 为 data
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)
    s -- model 相对于 data 的尺度, 即 data*s --> model
    """
    np.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)
    
    W = np.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += np.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    U,d,Vh = np.linalg.linalg.svd(W.transpose())
    S = np.matrix(np.identity( 3 ))
    if(np.linalg.det(U) * np.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U*S*Vh

    if calc_scale:
        rotmodel = rot*model_zerocentered
        dots = 0.0
        norms = 0.0
        for column in range(data_zerocentered.shape[1]):
            dots += np.dot(data_zerocentered[:,column].transpose(),rotmodel[:,column])
            normi = np.linalg.norm(model_zerocentered[:,column])
            norms += normi*normi
        # s = float(dots/norms)  
        s = float(norms/dots)
    else:
        s = 1.0  

    # trans = data.mean(1) - s*rot * model.mean(1)
    # model_aligned = s*rot * model + trans
    # alignment_error = model_aligned - data

    # scale the est to the gt, otherwise the ATE could be very small if the est scale is small
    trans = s*data.mean(1) - rot * model.mean(1)
    model_aligned = rot * model + trans
    data_alingned = s * data
    alignment_error = model_aligned - data_alingned
    
    trans_error = np.sqrt(np.sum(np.multiply(alignment_error,alignment_error),0)).A[0]
        
    return rot,trans,trans_error, s

def SE3toQT(RT):
    """
    Convert transformation matrix to quaternion and translation. (tum 格式)
    x y z i j k w
    """
    R, T = RT[:3, :3], RT[:3, 3]
    rot = Matrix(R)
    quadw = rot.to_quaternion() # w i j k https://docs.blender.org/api/current/mathutils.html#mathutils.Quaternion
    # quad = quadw[1, 2, 3, 0] # i j k w
    quad = [quadw[1], quadw[2], quadw[3], quadw[0]]
    tq = np.concatenate([T, quad], 0) # x y z i j k w
    return tq

def load_traj(gtpath, save=None):
    with open(gtpath, "r") as f:
        lines = f.readlines()
    n_img = len(lines)
    gtpose = []
    for i in range(n_img):
        timestamp = float(i * 1.0/fps) # 根据帧率设置伪时间
        line = lines[i]
        c2w = np.array(list(map(float, line.split()))).reshape(4, 4)
        # 转为tum
        tq = SE3toQT(c2w)
        ttq = np.concatenate([np.array([timestamp]), tq], 0) #  带上时间戳
        gtpose += [ttq]
    
    gtposes = np.stack(gtpose, 0) # (N, 8)
    
    if save is not None:
        np.savetxt(save, gtposes)
        print('save tum-format gt: {}'.format(save))
        
    return gtposes
        
def associate(first_list, second_list, offset=0.0, max_difference=0.02):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim
    to find the closest match for every input tuple.

    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation

    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))

    """
    first_keys = list(first_list.keys())
    second_keys = list(second_list.keys())
    potential_matches = [(abs(a - (b + offset)), a, b)
                         for a in first_keys
                         for b in second_keys
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))

    matches.sort()
    return matches       
        

def correct_mapscale(pcdfile, predpose, gttraj):
    """
    在3d点云上 进行 尺度校正
    pcdfile: 点云txt 文件路径
    predpose: 和上面点云对应的orb-slam2 的位姿 (tum 格式)
    gttraj: office0 序列真值 注意 Nx16 的格式
    """
    pcdarr = np.loadtxt(pcdfile)
    npt = pcdarr.shape[0]
    
    print('load point cloud from {} \nsize: {},{}'.format(pcdfile, npt, pcdarr.shape[1]))
    print('load pred pose from {}'.format(predpose))
    print('load gt traj from {}'.format(gttraj))
    
    gtpose = load_traj(gttraj, save='dataset/Replica/office0/tum_gt.txt')
    estpose = np.loadtxt(predpose)
    # 转变为 字典 key 为 时间戳
    n_gt = gtpose.shape[0]
    n_est = estpose.shape[0]
    print('gt pose: ', gtpose.shape)
    print('est pose: ', estpose.shape)
    dic_gt = dict([(gtpose[i, 0], gtpose[i, 1:4]) for i in range(n_gt)])
    dic_est = dict([(estpose[i, 0], estpose[i, 1:4]) for i in range(n_est)])
    matches = associate(dic_gt, dic_est)
    if len(matches) < 2:
        raise ValueError(
            "Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! \
            Did you choose the correct sequence?")
    first_xyz = np.matrix(
        [[float(value) for value in dic_gt[a][0:3]] for a, b in matches]).transpose()
    second_xyz = np.matrix([[float(value) for value in dic_est[b][0:3]] for a, b in matches]).transpose()
    
    # 对齐 得到尺度变换
    rot, trans, trans_error, s = align(first_xyz, second_xyz) # RT把前者 变为后者, s 把后者变前者
    # print('aligned ATE(m): {}'.format(trans_error))
    if True:
        print("compared_pose_pairs %d pairs" % (len(trans_error)))

        print("absolute_translational_error.rmse %f m" % np.sqrt(
            np.dot(trans_error, trans_error) / len(trans_error)))
        print("absolute_translational_error.mean %f m" %
              np.mean(trans_error))
        print("absolute_translational_error.median %f m" %
              np.median(trans_error))
        print("absolute_translational_error.std %f m" % np.std(trans_error))
        print("absolute_translational_error.min %f m" % np.min(trans_error))
        print("absolute_translational_error.max %f m" % np.max(trans_error))
    scale = float(s) # float(1./s)
    print('est map should x {}'.format(scale))
    
    correct_mapts = pcdarr * scale
    
    # 对比前后 点云 的 bound
    # print('[raw pcd] x y z min:\n', pcdarr.min(axis=0))
    # 保存新点云
    savescaleptfile = 'office0_orbalign_mappts.txt'
    np.savetxt(savescaleptfile, correct_mapts)
    print('save scaled map 3d pts: {}'.format(savescaleptfile))
    
    # 转为pcd 来可视化对比
    rawpcd = o3d.geometry.PointCloud()
    rawpcd.points = o3d.utility.Vector3dVector(pcdarr[:, :3])
    print('raw pcd box: \n', rawpcd.get_axis_aligned_bounding_box())
    saclepcd = o3d.geometry.PointCloud()
    saclepcd.points = o3d.utility.Vector3dVector(correct_mapts[:, :3])
    saclepcd.normals = o3d.utility.Vector3dVector(correct_mapts[:, :3] / np.linalg.norm(correct_mapts[:, :3], axis=-1, keepdims=True))
    print('scaled pcd box: \n', saclepcd.get_axis_aligned_bounding_box())
    
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.2, origin=[0, 0, 0]) #显示坐标系 1.0 20.0
    vis_lst = [rawpcd, saclepcd, mesh_frame]
    o3d.visualization.draw_geometries(vis_lst)
    
    
    

if __name__ == '__main__':
    # parser command lines
    parser = argparse.ArgumentParser(description='''
      
    ''') 
    parser.add_argument('--pcdfile', type=str, help='3d 点文件路径',default="office0_orb_mappts.txt") #
    parser.add_argument('--predpose', type=str, help='orb 估计的位姿路径',default="KeyFrameTrajectory.txt")
    parser.add_argument('--gttraj', type=str, help='office0 gt traj',default="dataset/Replica/office0/traj.txt")

    args = parser.parse_args()
    # 读取参数
    pcdfile = args.pcdfile
    predpose = args.predpose
    gttraj = args.gttraj
    # print("\n report result...\n")
    correct_mapscale(pcdfile, predpose, gttraj)