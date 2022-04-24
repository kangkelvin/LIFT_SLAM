import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

def match_transform(gt_ts,gt_h,pred_t0,pred_h0):
    # Inputs:
    # gt_ts is a list of times for ground truth frames
    # gt_h is an array of ground truth homogeneous matrices (n x 4 x 4)
    # pred_t0 is the time of the first predicted key frame
    # pred_h0 is the homogeneous matrix of the first predicted key frame
    # Return:
    # match_transform is a homogeneous transform that matches the predicted data to the ground truth data
    assert gt_ts[0] <= pred_t0
    gt_t0_indice = 0
    for ii in range(len(gt_ts)):
        if gt_ts[ii] >= pred_t0:
            gt_t0_indice = ii
            break
    if gt_ts[gt_t0_indice] != pred_t0:
        gt_h_matched = interpolate(pred_t0, gt_ts[gt_t0_indice-1], gt_ts[gt_t0_indice], gt_h[gt_t0_indice-1], gt_h[gt_t0_indice])
    else:
        gt_h_matched = gt_h[gt_t0_indice]
    match_transform = np.zeros((4,4), dtype=np.float64)
    match_transform[3,3] = 1.0
    match_transform[0:3,3] = gt_h_matched[0:3,3] - pred_h0[0:3,3]
    gt_r = R.from_matrix(gt_h_matched[0:3,0:3])
    pred_r = R.from_matrix(pred_h0[0:3,0:3])
    match_transform[0:3,0:3] = (gt_r * pred_r.inv()).as_matrix()
    return match_transform

def interpolate(tm,t1,t2,h1,h2):
    # Inputs:
    # tm is the time to match
    # t1 is the time of frame 1
    # t2 is the time of frame 2
    # h1 is the homogeneous matrix of frame 1
    # h2 is the homogeneous matrix of frame 2
    # Return:
    # hm is the interpolated, reduced (3x4) homogeneous matrix at tm
    assert t1 != t2
    if t1 > t2:
        t1, t2, h1, h2 = t2, t1, h2, h1
    assert tm > t1 and tm < t2
    w1 = (tm - t1) / (t2 - t1)
    hm = np.zeros((4,4), dtype=np.float64)
    hm[3,3] = 1.0
    # Interpolate translation
    hm[0:3,-1] = w1 * (h2[0:3,-1] - h1[0:3,-1]) + h1[0:3,-1]
    # Interpolate rotation
    key_rots = R.from_matrix([h1[0:3,0:3], h2[0:3,0:3]])
    key_times = [t1,t2]
    slerp = Slerp(key_times, key_rots)
    hm[0:3,0:3] = slerp(tm).as_matrix()
    return hm

if __name__ == "__main__":
    # Unit test interpolate function
    t1, t2, tm = 5, 1, 3
    h2 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])
    h1 = np.array([
        [-1, 0, 0, 0],
        [0, 1, 0, 4],
        [0, 0, -1, 0],
        [0, 0, 0, 1]])
    gt_hm = np.array([
        [0, 0, -1, 0],
        [0, 1, 0, 2],
        [1, 0, 0, 0],
        [0, 0, 0, 1]])
    hm = interpolate(tm, t1, t2, h1, h2)
    assert gt_hm.all() == hm.all()
    print("interpolate works")

    # Unit test match transform
    gt_ts = [0, 1, 2, 3, 4, 5]
    pred_t0 = 1
    gt_h = np.zeros((6,4,4), dtype=np.float64)
    gt_h[:,3,3] = 1.0
    gt_h[1,:,:] = gt_hm
    pred_h0 = np.identity(4, dtype=np.float64)
    transform = match_transform(gt_ts,gt_h,pred_t0,pred_h0)
    assert (transform @ pred_h0).all() == gt_h[1,:,:].all()
    gt_h[4,:,:] = h2
    gt_h[5,:,:] = h1
    pred_t0 = 4.5
    transform = match_transform(gt_ts, gt_h, pred_t0, pred_h0)
    assert (transform @ pred_h0).all() == gt_hm.all()
    assert (transform @ pred_h0).all() == (interpolate(pred_t0, gt_ts[4], gt_ts[5], gt_h[4], gt_h[5])).all()
    print("match_transform works")