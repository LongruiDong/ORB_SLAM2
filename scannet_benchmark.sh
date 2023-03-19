#!/bin/bash
# 在/home/dlr/Project/ORB_SLAM2 下！
pathdatadir='data/scannet_part'
#------------------------------------
# 按需求添加序列 注意 x.yaml 也要新建并更新内参
evalset=(
    scene0086_00
    scene0149_00
    scene0203_00
    scene0304_00
    scene0329_02
    scene0334_02
    scene0382_00
    scene0414_00
    scene0432_01
    scene0488_00
    scene0490_00
    scene0607_00
    scene0621_00
    scene0670_00
    scene0671_00
    scene0671_01
)
# tum 格式的 关键帧位姿 保存在 result/mono/scannet_${evalset[$i]}_KeyFrameTrajectory.txt
for ((i=0; i<16; i++)); do
    /home/dlr/Project/ORB_SLAM2/Examples/Monocular/mono_scannet /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Monocular/${evalset[$i]}.yaml $pathdatadir/${evalset[$i]}/
done


# eval


