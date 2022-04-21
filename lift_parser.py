from sys import path_importer_cache
from lift_features import get_lift_features
import numpy as np
import cv2
import os

if __name__ == '__main__':
    ### Parameters
    main_dir = "data/04-Straight-Line-Drive"
    ###

    img_dir = os.path.join(main_dir, "image_0/")
    kp_dir = os.path.join(main_dir, "kp_0/")
    desc_dir = os.path.join(main_dir, "desc_0/")

    if not os.path.exists(kp_dir):
        os.makedirs(kp_dir)
    if not os.path.exists(desc_dir):
        os.makedirs(desc_dir)

    for image_path in os.listdir(img_dir):
        full_path = os.path.join(img_dir, image_path)
        query_img = cv2.imread(full_path)
        query_img_bw = cv2.cvtColor(query_img, cv2.COLOR_BGR2GRAY)
        _, kp_list, desc = get_lift_features(query_img_bw, convert_to_uint8=True)
        kp_arr = np.stack(kp_list)
        # kp_arr = np.load("data/lift_keypoints_sample.npy")
        # desc = np.load("data/lift_descriptors_sample.npy")
        out_kp_path = os.path.join(kp_dir, image_path)
        out_desc_path = os.path.join(desc_dir, image_path)
        fs_kp = cv2.FileStorage(str(out_kp_path[:-4]) + ".xml", cv2.FILE_STORAGE_WRITE)
        fs_desc = cv2.FileStorage(str(out_desc_path[:-4]) + ".xml", cv2.FILE_STORAGE_WRITE)
        fs_kp.write("kp", kp_arr)
        fs_desc.write("desc", desc)
        # np.savetxt(str(out_kp_path[:-4]) + ".txt", kp_arr)
        # np.savetxt(str(out_desc_path[:-4]) + ".txt", desc)
        fs_kp.release()
        fs_desc.release()
