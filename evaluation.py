import numpy as np
import os
import argparse
from scipy.spatial.transform import Rotation as R

def scientificNotation2Value(str_in):
    if str_in[-1] == '\n':
        str_in[-1] == "\0"
    return float(str_in[:8])*(10**int(str_in[-3:]))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Evaluation SLAM Keyframes on KITTI Ground Truth')
    parser.add_argument('--sequence', type=int, default=4)
    parser.add_argument('--feature_type', type=str, default="LIFT")
    args = parser.parse_args()
    
    path2KITTI = "../KITTI/"
    KITTI2Poses = "dataset/poses/{:02d}.txt".format(args.sequence)
    KITTI2Times = "dataset/sequences-odometry-gray/{:02d}/times.txt".format(args.sequence)
    
    if args.feature_type == "LIFT":
        fpresults = "results/LIFT_SLAM/{:02d}.txt".format(args.sequence)
    if args.feature_type == "ORB":
        fpresults = "results/ORB_SLAM2_OG/{:02d}.txt".format(args.sequence)
    
    fpposes = os.path.join(path2KITTI,KITTI2Poses)
    fptimes = os.path.join(path2KITTI,KITTI2Times)
    
    gt_poses = [[]]
    with open(fpposes,"rb") as fp:
        for line in fp:
            data_str = line.split()
            gt_pose = []
            for element in data_str:
                gt_pose.append(scientificNotation2Value(element))
            pose = np.array([[gt_pose[0], gt_pose[1], gt_pose[2], gt_pose[3]],
                         [gt_pose[4], gt_pose[5], gt_pose[6], gt_pose[7]],
                         [gt_pose[8], gt_pose[9], gt_pose[10], gt_pose[11]],
                          [0,0,0,1]])
            gt_poses.append(pose)
    gt_poses = gt_poses[1:]        
    
    times = []
    with open(fptimes,"rb") as fp:
        for line in fp:
            time = scientificNotation2Value(line)
            times.append(time)
            
    result_poses = [[]]
    result_times = []
    with open(fpresults,"rb") as fp:
        for line in fp:
            data_str = line.split()
            data = []
            for element in data_str:
                data.append(float(element))
            result_times.append(data[0])
            r = R.from_quat([data[4], data[5], data[6], data[7]])
            pose = np.array([[0, 0, 0, data[1]],
                         [0, 0, 0, data[2]],
                         [0, 0, 0, data[3]],
                          [0,0,0,1]])
            pose[0:3,0:3] = r.as_matrix()
            result_poses.append(pose)
    result_poses = result_poses[1:]
