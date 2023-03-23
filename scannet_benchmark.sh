#!/bin/bash
# 在/home/dlr/Project/ORB_SLAM2 下！
pathdatadir='data/scannet_part2'
#------------------------------------
# 按需求添加序列 注意 x.yaml 也要新建并更新内参
evalset=(
    scene0204_01
    scene0300_01
    scene0329_01
    scene0329_02
    scene0473_01
    scene0520_01
    scene0561_00
    scene0639_00
    scene0678_02
    scene0692_04
    scene0701_02
)
# tum 格式的 关键帧位姿 保存在 result/mono/scannet_${evalset[$i]}_KeyFrameTrajectory.txt
# tracking 位姿在 result/mono/scannet_${evalset[$i]}_FrameTrajectory.txt
for ((i=0; i<16; i++)); do
    /home/dlr/Project/ORB_SLAM2/Examples/Monocular/mono_scannet /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Monocular/${evalset[$i]}.yaml $pathdatadir/${evalset[$i]}/
done


# eval


