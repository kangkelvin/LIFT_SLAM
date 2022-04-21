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
std::vector<cv::KeyPoint> ReadKeypointFromXml(std::string base_dir, std::string image_id) {
    cv::FileStorage fs;
    base_dir.append("/kp_0/");
    base_dir.append(image_id);
    base_dir.append(".xml");
    fs.open(base_dir, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "error in reading file: " << base_dir << std::endl;
    }

    cv::Mat placeholder;
    fs["kp"] >> placeholder;

    int n_keypoints = placeholder.rows;

    std::vector<cv::KeyPoint> out;

    for (int r = 0; r < n_keypoints; r++) {
        cv::KeyPoint kp;
        kp.pt.x = placeholder.at<double>(r, 0);
        kp.pt.y = placeholder.at<double>(r, 1);
        kp.size = placeholder.at<double>(r, 2) * 2.0;
        kp.angle = placeholder.at<double>(r, 3);
        kp.response = placeholder.at<double>(r, 4);
        // kp.octave = placeholder.at<double>(r, 5);
        kp.octave = 0.0;
        kp.class_id = -1;

        out.push_back(kp);
    }

    return out;
}

cv::Mat ReadDescFromXml(std::string base_dir, std::string image_id) {
    cv::FileStorage fs;
    base_dir.append("/desc_0/");
    base_dir.append(image_id);
    base_dir.append(".xml");
    fs.open(base_dir, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "error in reading file: " << base_dir << std::endl;
    }

    cv::Mat out;
    fs["desc"] >> out;

    return out;
}
