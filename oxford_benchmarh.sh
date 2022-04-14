#!/bin/bash
pathDatasetRadarRobot='/media/dlr/nd/oxfordradar' #for amax

mkdir logs
mkdir result/stereotrack -p
mkdir result/stereo -p
#------------------------------------
# Stereo-Lidar /home/dlr/Project/ORB_SLAM2/Examples


echo "Launching oxford 1 with Stereo sensor" #imlstereo_oxford stereo_oxford  addcov 270 120
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_oxford /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/RadarRbtcar.yaml "$pathDatasetRadarRobot"/2019-01-10-14-36-48-radar-oxford-10k-partial 1 0 8731

echo "Launching oxford 2 with Stereo sensor" # 0 800
# /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_oxford /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/RadarRbtcar.yaml "$pathDatasetRadarRobot"/2019-01-10-14-36-48-radar-oxford-10k-partial 2 0 3731

nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_oxford /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/RadarRbtcar.yaml "$pathDatasetRadarRobot"/2019-01-10-14-36-48-radar-oxford-10k-partial 1 0 8731 >logs/hzqout1.txt 2>&1 &
nohup /home/dlr/Project/ORB_SLAM2/Examples/Stereo/stereo_oxford /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Stereo/RadarRbtcar.yaml "$pathDatasetRadarRobot"/2019-01-10-14-36-48-radar-oxford-10k-partial 2 0 3731 >logs/hzqout2.txt 2>&1 &


# eval  --useSE 3 --seqs 2
# python ~/Project/kitti-odom-eval/eval_oxford.py --align 6dof --result result/stereo
# python ~/Project/kitti-odom-eval/eval_oxford.py --align 6dof --result result/stereotrack

