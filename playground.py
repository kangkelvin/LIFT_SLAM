from sys import path_importer_cache
from lift_features import get_lift_features
import numpy as np
import cv2
import os

if __name__ == '__main__':
    # f_path_q = "data/06-2U-turns-same-road/image_0/000000.png"
    # query_img = cv2.imread(f_path_q)
    # query_img_bw = cv2.cvtColor(query_img, cv2.COLOR_BGR2GRAY)
    # _, queryKeypoints, queryDescriptors = get_lift_features(query_img_bw)
    # np.save("data/lift_keypoints_sample.npy", queryKeypoints)
    # np.save("data/lift_descriptors_sample.npy", queryDescriptors)

    # queryKeypoints = np.load("data/lift_keypoints_sample.npy")
    # queryDescriptors = np.load("data/lift_descriptors_sample.npy")
    
    # print(queryDescriptors[5])
    # print(np.min(queryDescriptors)) 
    # print(np.max(queryDescriptors))

    # print(queryKeypoints[0])

    img_dir = "data/06-2U-turns-same-road/image_0"
    for image_path in os.listdir(img_dir):
        full_path = os.path.join(img_dir, image_path)
        print(full_path)
    pass
