from copy import deepcopy

import cv2
from scipy.stats import boxcox
import numpy as np

from utils.lift_utils.custom_types import paramGroup, paramStruct, pathConfig
from utils.lift_utils.dump_tools import loadh5
from utils.lift_utils.filter_tools import apply_learned_filter_2_image_no_theano
from utils.lift_utils.kp_tools import XYZS2kpList, get_XYZS_from_res_list, kp_list_2_opencv_kp_list, IDX_ANGLE, update_affine
# from utils.lift_utils.sift_tools import recomputeOrientation
from utils.lift_utils.solvers import Test
from utils.lift_utils.dataset_tools.data_obj import data_obj

def get_lift_features(img_in: np.ndarray, convert_to_uint8: False):
    """
    img_in: grayscale image in np array format (H x W)
    return: image_overlay (colored img), keypoint location, keypoints descriptions
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
    # kp_list, _ = recomputeOrientation(img_in, kp_list,
    #                                       bSingleOrientation=True)

    ##############################################################
    # Compute Orientation with Learned Features
    # Setup
    param = paramStruct()
    param.loadParam(config_file, verbose=False)
    pathconf = pathConfig()
    pathconf.setupTrain(param, 0)
    pathconf.result = model_dir

    param.model.sDetector = 'bypass'
    # This ensures that you don't create unecessary scale space
    param.model.fScaleList = np.array([1.0])
    param.patch.fMaxScale = np.max(param.model.fScaleList)
    # this ensures that you don't over eliminate features at boundaries
    param.model.nPatchSize = int(np.round(param.model.nDescInputSize) *
                                 np.sqrt(2))
    param.patch.fRatioScale = (float(param.patch.nPatchSize) /
                               float(param.model.nDescInputSize)) * 6.0
    param.model.sDescriptor = 'bypass'

    # add the mean and std of the learned model to the param
    mean_std_file = pathconf.result + 'mean_std.h5'
    mean_std_dict = loadh5(mean_std_file)
    param.online = paramGroup()
    setattr(param.online, 'mean_x', mean_std_dict['mean_x'])
    setattr(param.online, 'std_x', mean_std_dict['std_x'])

    # -------------------------------------------------------------------------
    # Load data in the test format
    kp_array = np.stack(kp_list)
    test_data_in = data_obj(param, img_in, kp_array)

    # -------------------------------------------------------------------------
    # Test using the test function
    _, oris, compile_time = Test(
        pathconf, param, test_data_in, test_mode="ori")

    # update keypoints and save as new
    kp_array = test_data_in.coords
    for idxkp in range(kp_array.shape[0]):
        kp_array[idxkp][IDX_ANGLE] = oris[idxkp] * 180.0 / np.pi % 360.0
        kp_array[idxkp] = update_affine(kp_array[idxkp])

    ##############################################################
    # Compute descriptor
    # Setup
    param = paramStruct()
    param.loadParam(config_file, verbose=False)
    pathconf = pathConfig()
    pathconf.setupTrain(param, 0)
    pathconf.result = model_dir

    setattr(param.model, "descriptor_export_folder", "models/base")

    # Modify the network so that we bypass the keypoint part and the
    # orientation part
    param.model.sDetector = 'bypass'
    # This ensures that you don't create unecessary scale space
    param.model.fScaleList = np.array([1.0])
    param.patch.fMaxScale = np.max(param.model.fScaleList)
    # this ensures that you don't over eliminate features at boundaries
    param.model.nPatchSize = int(np.round(param.model.nDescInputSize) *
                                 np.sqrt(2))
    param.patch.fRatioScale = (float(param.patch.nPatchSize) /
                               float(param.model.nDescInputSize)) * 6.0
    param.model.sOrientation = 'bypass'

    # add the mean and std of the learned model to the param
    mean_std_file = pathconf.result + 'mean_std.h5'
    mean_std_dict = loadh5(mean_std_file)
    param.online = paramGroup()
    setattr(param.online, 'mean_x', mean_std_dict['mean_x'])
    setattr(param.online, 'std_x', mean_std_dict['std_x'])

    # -------------------------------------------------------------------------
    # Load data in the test format
    test_data_in = data_obj(param, img_in, kp_array)

    # -------------------------------------------------------------------------
    # Test using the test function
    descs, _, compile_time = Test(
        pathconf, param, test_data_in, test_mode="desc")

    if convert_to_uint8:
        descs = descs * 255.0 / 4.0
        # descs = (-1 * descs + 4.0)
        # descs = np.log(descs)
        # descs = (descs + 6.0) * 255.0 / 8.0
        descs = np.clip(descs, 0, 255)
        descs = np.array(descs, dtype=np.ubyte)

    kp_list = []
    for i in range(kp_array.shape[0]):
        kp_list.append(kp_array[i])

    return image_color, kp_list, descs

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
    seq_name = "04-Straight-Line-Drive"
    f_path_q = "data/" + seq_name + "/image_0/000101.png"
    f_path_t = "data/" + seq_name + "/image_0/000100.png"   
    
    # Read image
    query_img = cv2.imread(f_path_q)
    train_img = cv2.imread(f_path_t)
    # Optional trim to make width smaller
    query_img = query_img[:, 400:801, :]
    train_img = train_img[:, 400:801, :]

    # Convert it to grayscale
    query_img_bw = cv2.cvtColor(query_img, cv2.COLOR_BGR2GRAY)
    train_img_bw = cv2.cvtColor(train_img, cv2.COLOR_BGR2GRAY)
    
    ### Use ORB
    # orb = cv2.ORB_create()
    # queryKeypoints, queryDescriptors = orb.detectAndCompute(query_img_bw,None)
    # trainKeypoints, trainDescriptors = orb.detectAndCompute(train_img_bw,None)

    ### Use Lift
    _, queryKeypoints, queryDescriptors = get_lift_features(query_img_bw, True)
    _, trainKeypoints, trainDescriptors = get_lift_features(train_img_bw, True)
    queryKeypoints = kp_list_2_opencv_kp_list(queryKeypoints)
    trainKeypoints = kp_list_2_opencv_kp_list(trainKeypoints)

    # import matplotlib.pyplot as plt
    # n, bins, patches = plt.hist(x=queryDescriptors.flatten(), bins='auto', color='#0504aa',
    #                             alpha=0.7)
    # plt.grid(axis='y', alpha=0.75)
    # plt.xlabel('Value')
    # plt.ylabel('Frequency')
    # plt.show()

    ### See features only
    # query_img = cv2.drawKeypoints(query_img_bw, queryKeypoints[:500], query_img)

    # Initialize the Matcher for matching
    matcher = cv2.BFMatcher()

    ### Use simple brute force
    matches = matcher.match(queryDescriptors,trainDescriptors)
    final_img = cv2.drawMatches(query_img, queryKeypoints, train_img, trainKeypoints, matches[:40], None)

    ### Use FLANN
    # FLANN_INDEX_KDTREE = 1
    # index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    # search_params = dict(checks = 50)
    # flann = cv2.FlannBasedMatcher(index_params, search_params)
    # matches = flann.knnMatch(queryDescriptors,trainDescriptors,k=2)

    # good = []
    # for m,n in matches:
    #     good.append([m])

    # final_img = cv2.drawMatchesKnn(query_img, queryKeypoints, train_img, trainKeypoints, good[:40], None, flags=2)
    
    # Show the final image
    cv2.imshow("Matches", final_img)
    cv2.waitKey(0) 
    
    #closing all open windows 
    cv2.destroyAllWindows() 
