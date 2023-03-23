#!/bin/bash
# 在/home/dlr/Project/ORB_SLAM2 下！
pathdatadir='data/TUM' # data/TUM/rgbd_dataset_ freiburg1_360
#------------------------------------
# 按需求添加序列 对于TUM 只需要三个内参文件

evalset1=( # freiburg1
    360
    desk
    desk2
    floor
    room
    
)
# tum 格式的 关键帧位姿 保存在 result/mono/rgbd_dataset_freiburg1_${evalset[$i]}_KeyFrameTrajectory.txt
# tracking 位姿在 result/mono/rgbd_dataset_freiburg1_${evalset[$i]}_FrameTrajectory.txt
for ((i=0; i<5; i++)); do
    /home/dlr/Project/ORB_SLAM2/Examples/Monocular/mono_tum /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Monocular/TUM1.yaml $pathdatadir/rgbd_dataset_freiburg1_${evalset1[$i]}/
done

# evalset2=( # freiburg2
#     360_kidnap
#     desk
#     large_no_loop
#     large_with_loop
#     xyz
# )
# # tum 格式的 关键帧位姿 保存在 result/mono/rgbd_dataset_freiburg2_${evalset[$i]}_KeyFrameTrajectory.txt
# # tracking 位姿在 result/mono/rgbd_dataset_freiburg2_${evalset[$i]}_FrameTrajectory.txt
# for ((i=0; i<5; i++)); do
#     /home/dlr/Project/ORB_SLAM2/Examples/Monocular/mono_tum /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Monocular/TUM2.yaml $pathdatadir/rgbd_dataset_freiburg2_${evalset2[$i]}/
# done


# evalset3=( # freiburg3
#     cabinet
#     long_office_household
# )
# # tum 格式的 关键帧位姿 保存在 result/mono/rgbd_dataset_freiburg3_${evalset[$i]}_KeyFrameTrajectory.txt
# # tracking 位姿在 result/mono/rgbd_dataset_freiburg3_${evalset[$i]}_FrameTrajectory.txt
# for ((i=0; i<2; i++)); do
#     /home/dlr/Project/ORB_SLAM2/Examples/Monocular/mono_tum /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Monocular/TUM3.yaml $pathdatadir/rgbd_dataset_freiburg3_${evalset3[$i]}/
# done

# eval


