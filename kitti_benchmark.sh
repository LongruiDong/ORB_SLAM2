#!/bin/bash
#raw data extract - benchmark /media/kitti
pathDatasetKITTI='/home/dlr/kitti/dataset/sequences' #for amax44
# pathDatasetKITTI='/mdata/kitti/dataset/sequences' #for PC
# 00 01 02 03 08 09 10 的平移需要降低 
# 00 02 03 04 05 07 08 09 10 的旋转需要降低
# 00 02 05 06 07 09 contain loops
mkdir logs
mkdir result/stereotrack -p
mkdir result/stereo -p
#------------------------------------
# Stereo-Lidar /home/dlr/Project/ORB_SLAM2/Examples


# echo "Launching KITTI-04 with Stereo-Lidar sensor" #imlstereo_kitti stereo_kitti  addcov 270 120
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/04 /home/dlr/Project/ORB_SLAM2/rectified_imu_kitti/04.txt 0 270 0 > logs/04log.txt

# echo "Launching KITTI-03 with Stereo-Lidar sensor" # 0 800
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI03.yaml "$pathDatasetKITTI"/03 /home/dlr/Project/ORB_SLAM2/rectified_imu_kitti/03.txt 0 800 0 > logs/03log.txt


# echo "Launching KITTI-07 with Stereo-Lidar sensor"
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/07 /home/dlr/Project/ORB_SLAM2/rectified_imu_kitti/07.txt 0 1100 0 > logs/07log.txt

# echo "Launching KITTI-10 with Stereo-Lidar sensor"
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/10 /home/dlr/Project/ORB_SLAM2/rectified_imu_kitti/10.txt 0 1200 0 > logs/10log.txt

# echo "Launching KITTI-06 with Stereo-Lidar sensor"
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/06 /home/dlr/Project/ORB_SLAM2/rectified_imu_kitti/06.txt 0 1100 0 > logs/06log.txt

# echo "Launching KITTI-09 with Stereo-Lidar sensor"
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/09 /home/dlr/Project/ORB_SLAM2/rectified_imu_kitti/09.txt 0 1590 0 > logs/09log.txt

# echo "Launching KITTI-02 with Stereo-Lidar sensor" #0 4360 4660
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI00-02.yaml "$pathDatasetKITTI"/02 /home/dlr/Project/ORB_SLAM2/rectified_imu_kitti/02.txt 0 4660 0 > logs/02log.txt

# echo "Launching KITTI-05 with Stereo-Lidar sensor"
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/05 /home/dlr/Project/ORB_SLAM2/rectified_imu_kitti/05.txt 0 2760 0 > logs/05log.txt

# echo "Launching KITTI-08 with Stereo-Lidar sensor" #4070 203 3750 4050
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/08 /home/dlr/Project/ORB_SLAM2/rectified_imu_kitti/08.txt 0 4070 0 > logs/08log.txt

# echo "Launching KITTI-00 with Stereo-Lidar sensor"
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI00-02.yaml "$pathDatasetKITTI"/00 /home/dlr/Project/ORB_SLAM2/rectified_imu_kitti/00.txt 0 4540 0 > logs/00log.txt

# echo "Launching KITTI-01 with Stereo-Lidar sensor"
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI00-02.yaml "$pathDatasetKITTI"/01 /home/dlr/Project/ORB_SLAM2/rectified_imu_kitti/01.txt 0 1100 0 > logs/01log.txt

nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/04 >logs/hzqout4.txt 2>&1 &
nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI03.yaml "$pathDatasetKITTI"/03 >logs/hzqout3.txt 2>&1 &
nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/08 >logs/hzqout8.txt 2>&1 &
nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI00-02.yaml "$pathDatasetKITTI"/02 >logs/hzqout2.txt 2>&1 &
nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/07 >logs/hzqout7.txt 2>&1 &
nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/10 >logs/hzqout10.txt 2>&1 &
nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/06 >logs/hzqout6.txt 2>&1 &
nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/09 >logs/hzqout9.txt 2>&1 &
nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetKITTI"/05 >logs/hzqout5.txt 2>&1 &
nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI00-02.yaml "$pathDatasetKITTI"/00 >logs/hzqout0.txt 2>&1 &
nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_kitti /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI00-02.yaml "$pathDatasetKITTI"/01 >logs/hzqout1.txt 2>&1 &

# eval
python ~/Project/kitti-odom-eval/eval_odom.py --align 6dof --result result/stereo
python ~/Project/kitti-odom-eval/eval_odom.py --align 6dof --result result/stereotrack

