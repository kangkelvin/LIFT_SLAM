import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

def interpolate(tm,t1,t2,h1,h2):
    # Inputs:
    # tm is the time to match
    # t1 is the time of frame 1
    # t2 is the time of frame 2
    # h1 is the reduced (3x4) homogeneous matrix of frame 1
    # h2 is the reduced (3x4) homogeneous matrix of frame 2
    # Return:
    # hm is the interpolated, reduced (3x4) homogeneous matrix at tm
    assert t1 != t2
    if t1 > t2:
        t1, t2, h1, h2 = t2, t1, h2, h1
    assert tm > t1 and tm < t2
    w1 = (tm - t1) / (t2 - t1)
    w2 = 1.0 - w1
    hm = np.zeros((3,4), dtype=np.float64)
    # Interpolate translation
    hm[:,-1] = w1 * (h2[:,-1] - h1[:,-1]) + h1[:,-1]
    # Interpolate rotation
    key_rots = R.from_matrix([h1[:,0:3], h2[:,0:3]])
    key_times = [t1,t2]
    slerp = Slerp(key_times, key_rots)
    hm[:,0:3] = slerp(tm).as_matrix()
    return hm

if __name__ == "__main__":
    t1 = 5
    t2 = 1
    tm = 3
    h2 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0]])
    h1 = np.array([
        [-1, 0, 0, 0],
        [0, 1, 0, 4],
        [0, 0, -1, 0]])
    gt_hm = np.array([
        [0, 0, -1, 0],
        [0, 1, 0, 2],
        [1, 0, 0, 0]])
    hm = interpolate(tm, t1, t2, h1, h2)
    print("gt_hm: ", gt_hm)
    print("hm: ", hm)
    assert gt_hm.all() == hm.all()
    print("Success!")