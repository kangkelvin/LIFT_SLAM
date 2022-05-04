import numpy as np
import os
import argparse
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

def scientificNotation2Value(str_in):
    if str_in[-1] == '\n':
        str_in[-1] == "\0"
    return float(str_in[:8])*(10**int(str_in[-3:]))

# https://stackoverflow.com/questions/2566412/find-nearest-value-in-numpy-array
def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx, array[idx]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Evaluation SLAM Keyframes on KITTI Ground Truth')
    parser.add_argument('--sequence', type=int, default=6)
    parser.add_argument('--feature_type', type=str, default="LIFT")
    args = parser.parse_args()
    print(args)
    
    path2KITTI = "../KITTI/"
    KITTI2Poses = "dataset/poses/{:02d}.txt".format(args.sequence)
    KITTI2Times = "dataset/sequences-odometry-gray/{:02d}/times.txt".format(args.sequence)
    
    if args.feature_type == "LIFT":
        fpresults = "results/LIFT_SLAM/{:02d}.txt".format(args.sequence)
    if args.feature_type == "ORB":
        fpresults = "results/ORB_SLAM2_OG/{:02d}.txt".format(args.sequence)
    
    fpposes = os.path.join(path2KITTI,KITTI2Poses)
    fptimes = os.path.join(path2KITTI,KITTI2Times)
    
    # Load the ground truth poses and convert the lines into a 4x4 homogeneous transform
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
    
    # Load the time stamp associated with each ground truth frame
    gt_times = []
    with open(fptimes,"rb") as fp:
        for line in fp:
            gt_times.append(scientificNotation2Value(line))        
    
    # Load the resulting output time, position, and quaternion and load the
    #   times into time array and the poses (converted from position & quaternion
    #   to 4x4 homogeneous) into pose array so indices match up between them.
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
    
    # Get match transform:
    idx, value = find_nearest(gt_times,result_times[0])
    
    match_transform = np.zeros((4,4), dtype=np.float64)
    match_transform[3,3] = 1.0
    match_transform[0:3,3] = gt_poses[idx][0:3,3] - result_poses[0][0:3,3]
    gt_r = R.from_matrix(gt_poses[idx][0:3,0:3])
    pred_r = R.from_matrix(result_poses[0][0:3,0:3])
    match_transform[0:3,0:3] = (gt_r * pred_r.inv()).as_matrix()
    
    # Building A & b to solve for scaling factor with Ax=b
    A = np.array([])
    b = np.array([])
    result_xyz = [[]]
    gt_xyz = [[]]
    for frame in range(len(result_poses)):
        idx, value = find_nearest(gt_times,result_times[frame])
        
        transformed_result_pose = match_transform @ result_poses[frame]
        result_xyz.append(transformed_result_pose[0:3,3])
        gt_xyz.append(gt_poses[idx][0:3,3])
        
        A = np.append(A,transformed_result_pose[0:3,3])
        b = np.append(b,gt_poses[idx][0:3,3])
        
    A = A.reshape(-1,1)
    b = b.reshape(-1,1)
    x, res, rank, s = np.linalg.lstsq(A,b, rcond=None)
    
    l1norm = np.mean(np.abs(A*x-b))
    print("l1norm:", l1norm)
    
    result_xyz = np.array(result_xyz[1:])
    gt_xyz = np.array(gt_xyz[1:])
    
    l2norm = np.sqrt(np.mean(np.square(np.linalg.norm(result_xyz*x-gt_xyz,axis=1))))
    print("l2norm:", l2norm)
    