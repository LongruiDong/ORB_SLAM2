#!/bin/bash
# 在/home/dlr/Project/ORB_SLAM2 下！
pathdatadir='data/tcsvt'
#------------------------------------
# 按需求添加序列 注意 x.yaml 也要新建并更新内参
evalset=(
    atrium/A0
    atrium/A1
    atrium/A2
    atrium/A3
    atrium/A4
    corridor/C0
    corridor/C1
    corridor/C2
    corridor/C3
    exhibition-hall/E0
    exhibition-hall/E1
    exhibition-hall/E2
    indoor-office-room/I0
    indoor-office-room/I1
    indoor-office-room/I2
    outdoor-office-park/O0
    outdoor-office-park/O1
    outdoor-office-park/O2
    stairs/S0
    stairs/S1
    stairs/S2
    whole-floor/W0
    whole-floor/W1
    whole-floor/W2
    whole-floor/W3
    
)

seqset=(
    A0
    A1
    A2
    A3
    A4
    C0
    C1
    C2
    C3
    E0
    E1
    E2
    I0
    I1
    I2
    O0
    O1
    O2
    S0
    S1
    S2
    W0
    W1
    W2
    W3
)
# tum 格式的 关键帧位姿 保存在 result/mono/lsfb_flip_${evalset[$i]}_KeyFrameTrajectory.txt
for ((i=0; i<1; i++)); do
    /home/dlr/Project/ORB_SLAM2/Examples/Monocular/mono_lsfb /home/dlr/Project/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/dlr/Project/ORB_SLAM2/Examples/Monocular/android_${seqset[$i]}.yaml $pathdatadir/${evalset[$i]}/
done


# eval


