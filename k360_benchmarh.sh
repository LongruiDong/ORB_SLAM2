#!/bin/bash
#raw data extract - benchmark /media/kitti
# pathDatasetK360='/home/dlr/kitti/dataset/sequences' #for amax44
pathDatasetK360='/home/dlr/kt360/sequences' #for amax44

mkdir logs
mkdir result/stereotrack -p
mkdir result/stereo -p
#------------------------------------
# Stereo-Lidar /home/dlr/Project/ORB_SLAM2/Examples


# echo "Launching KITTI-04 with Stereo-Lidar sensor" #imlstereo_k360 stereo_k360  addcov 270 120
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_k360 /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI04-12.yaml "$pathDatasetK360"/04 /home/dlr/Project/ORB_SLAM2/rectified_imu_kitti/04.txt 0 270 0 > logs/04log.txt

# echo "Launching KITTI-03 with Stereo-Lidar sensor" # 0 800
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_k360 /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI03.yaml "$pathDatasetK360"/03 /home/dlr/Project/ORB_SLAM2/rectified_imu_kitti/03.txt 0 800 0 > logs/03log.txt
nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_k360 /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI360.yaml "$pathDatasetK360"/12 >logs/hzqout12.txt 2>&1 &
nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_k360 /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/KITTI360.yaml "$pathDatasetK360"/14 >logs/hzqout14.txt 2>&1 &


# eval
# python ~/Project/kitti-odom-eval/eval_odom.py --align 6dof --result result/stereo
# python ~/Project/kitti-odom-eval/eval_odom.py --align 6dof --result result/stereotrack

