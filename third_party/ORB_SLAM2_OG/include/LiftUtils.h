/**
 * @file LiftUtils.h
 * @author Kelvin Kang
 * @brief Helper function to integrate LIFT and ORB_SLAM2
 * @version 0.1
 * @date 2022-04-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <opencv2/opencv.hpp>
#include <string>
#include <stdio.h>
#include <vector>

/**
 * @brief read the keypoint from file
 * 
 * @param base_dir 
 * @param image_id 
 * @return cv::KeyPoint 
 */
std::vector<cv::KeyPoint> ReadKeypointFromXml(string base_dir, string image_id) {
    cv::FileStorage fs;
    base_dir.append(file_name);
    base_dir.append(".xml");
    fs.open(base_dir, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "error in reading file: " << base_dir << std::endl;
    }

    cv::Mat placeholder;
    fs["kp"] >> placeholder;

    cv::Size kps_size = placeholder.size()
    int n_keypoints = placeholder.rows;

    std::vector<cv::KeyPoint> out;

    for (r = 0; r < n_keypoints; r++) {
        cv::KeyPoint kp;
        kp.pt.x = placeholder[r][0];
        kp.pt.y = placeholder[r][1];
        kp.size = placeholder[r][2];
        kp.angle = placeholder[r][3];
        kp.response = placeholder[r][4];
        kp.octave = placeholder[r][5];
        kp.class_id = -1

        out.push_back(kp)
    }

    return out
}

cv::Mat ReadDescFromXml(string base_dir, string image_id) {
    static constexpr lift_to_orb_const = 255.0 / 4.0;
    
    cv::FileStorage fs;
    base_dir.append(file_name);
    base_dir.append(".xml");
    fs.open(base_dir, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "error in reading file: " << base_dir << std::endl;
    }

    cv::Mat placeholder;
    fs["desc"] >> placeholder;

    cv::Size desc_size = placeholder.size()
    int n_keypoints = placeholder.rows;
    int desc_len = placeholder.cols;

    cv::Mat out_8u = cv::Mat(desc_size, cv::CV_8U);

    for (int r = 0; r < n_keypoints; r++) {
        for (int c = 0; c < desc_len; c++) {
            out_8u[r][c] = std::max(std::min(int(placeholder[r][c] * lift_to_orb_const), 255), 0)
        }
    }

    return out_8u
}
