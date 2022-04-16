import os
import sys
import time
from copy import deepcopy

import cv2
import h5py
import numpy as np

from utils.lift_utils.custom_types import paramGroup, paramStruct, pathConfig
from utils.lift_utils.dump_tools import loadh5
from utils.lift_utils.filter_tools import apply_learned_filter_2_image_no_theano
from utils.lift_utils.kp_tools import XYZS2kpList, get_XYZS_from_res_list, saveKpListToTxt
from utils.lift_utils.sift_tools import recomputeOrientation
from utils.lift_utils.solvers import TestImage

def get_lift_features(img_in: np.ndarray):
    """
    img_in: grayscale image in np array format (H x W)
    return: image_overlay (colored img) and feature vector 
    """

    ### Parameters ###
    config_file = "models/configs/picc-finetune-nopair.config"
    model_dir = "models/picc-best/"
    num_keypoint = 1000
    ##################

    ##############################################################
    # Check that image is greyscale
    assert len(img_in.shape) == 2

    # Setup
    param = paramStruct()
    param.loadParam(config_file, verbose=False)
    pathconf = pathConfig()
    pathconf.setupTrain(param, 0)
    pathconf.result = model_dir
    image_color = cv2.cvtColor(img_in, cv2.COLOR_GRAY2RGB)

    # check size
    image_height = img_in.shape[0]
    image_width = img_in.shape[1]

    ##############################################################
    # Multiscale Testing
    scl_intv = getattr(param.validation, 'nScaleInterval', 4)
    # min_scale_log2 = 1  # min scale = 2
    # max_scale_log2 = 4  # max scale = 16
    min_scale_log2 = getattr(param.validation, 'min_scale_log2', 1)
    max_scale_log2 = getattr(param.validation, 'max_scale_log2', 4)
    # Test starting with double scale if small image
    min_hw = np.min(img_in.shape[:2])
    if min_hw <= 1600:
        # print("INFO: Testing double scale")
        min_scale_log2 -= 1
    # range of scales to check
    num_division = (max_scale_log2 - min_scale_log2) * (scl_intv + 1) + 1
    scales_to_test = 2**np.linspace(min_scale_log2, max_scale_log2,
                                    num_division)

    # convert scale to image resizes
    resize_to_test = ((float(param.model.nPatchSizeKp - 1) / 2.0) /
                      (param.patch.fRatioScale * scales_to_test))

    # check if resize is valid
    min_hw_after_resize = resize_to_test * np.min(img_in.shape[:2])
    is_resize_valid = min_hw_after_resize > param.model.nFilterSize + 1

    # if there are invalid scales and resizes
    if not np.prod(is_resize_valid):
        # find first invalid
        first_invalid = np.where(True - is_resize_valid)[0][0]

        # remove scales from testing
        scales_to_test = scales_to_test[:first_invalid]
        resize_to_test = resize_to_test[:first_invalid]

    # Run for each scale
    test_res_list = []
    for resize in resize_to_test:

        # Just designate only one scale to bypass resizing. Just a single
        # number is fine, no need for a specific number
        param_cur_scale = deepcopy(param)
        param_cur_scale.patch.fScaleList = [
            1.0
        ]

        # resize according to how we extracted patches when training
        new_height = np.cast['int'](np.round(image_height * resize))
        new_width = np.cast['int'](np.round(image_width * resize))
        image = cv2.resize(img_in, (new_width, new_height))

        sKpNonlinearity = getattr(param.model, 'sKpNonlinearity', 'None')
        test_res = apply_learned_filter_2_image_no_theano(
            image, pathconf.result,
            param.model.bNormalizeInput,
            sKpNonlinearity,
            verbose=True)

        test_res_list += [np.pad(test_res,
                                 int((param.model.nFilterSize - 1) / 2),
                                 # mode='edge')]
                                 mode='constant',
                                 constant_values=-np.inf)]

    ##############################################################
    # Non-max suppresion and draw.
    nearby = int(np.round(
        (0.5 * (param.model.nPatchSizeKp - 1.0) *
         float(param.model.nDescInputSize) /
         float(param.patch.nPatchSize))
    ))
    fNearbyRatio = getattr(param.validation, 'fNearbyRatio', 1.0)
    # Multiply by quarter to compensate
    fNearbyRatio *= 0.25
    nearby = int(np.round(nearby * fNearbyRatio))
    nearby = max(nearby, 1)

    nms_intv = getattr(param.validation, 'nNMSInterval', 2)
    edge_th = getattr(param.validation, 'fEdgeThreshold', 10)
    do_interpolation = getattr(param.validation, 'bInterpolate', True)

    fScaleEdgeness = getattr(param.validation, 'fScaleEdgeness', 0)
    res_list = test_res_list
    XYZS = get_XYZS_from_res_list(res_list, resize_to_test,
                                  scales_to_test, nearby, edge_th,
                                  scl_intv, nms_intv, do_interpolation,
                                  fScaleEdgeness)
    XYZS = XYZS[:num_keypoint]
    image_color = draw_XYZS_to_img(XYZS, image_color)

    ##############################################################
    # Sorted keypoint list
    kp_list = XYZS2kpList(XYZS)
    # Keypoint list with orientation computed
    new_kp_list, _ = recomputeOrientation(img_in, kp_list,
                                          bSingleOrientation=True)

    return image_color, new_kp_list

def draw_XYZS_to_img(XYZS, image_color):
        """
        Draw the features on the colored image
        """

        # draw onto the original image
        if cv2.__version__[0] == '3':
            linetype = cv2.LINE_AA
        else:
            linetype = cv2.LINE_AA
        [cv2.circle(image_color, tuple(np.round(pos).astype(int)),
                    np.round(rad * 1.0).astype(int), (0, 255, 0), 1,
                    lineType=linetype)
                    for pos, rad in zip(XYZS[:, :2], XYZS[:, 2])]

        return image_color

if __name__ == '__main__':
    f_path = "data/04-Straight-Line-Drive/image_0/000000.png"
    img_in = cv2.imread(f_path, cv2.IMREAD_GRAYSCALE)
    image_color, new_kp_list = get_lift_features(img_in)
