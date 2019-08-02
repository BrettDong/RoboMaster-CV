/****************************************************************************
 *  Copyright (C) 2019 Brett Dong
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#pragma once
#ifndef DETECTOR_H_
#define DETECTOR_H_

#include <string>
#include <atomic>
#include <thread>
#include <functional>
#include <opencv2/opencv.hpp>

class Detector
{
    private:
        std::vector<cv::Point3f> ArmorVertex2D;
        cv::Mat intrinsic, distortion;
        cv::VideoCapture cap;
        std::atomic<bool> running;
        std::function<bool(bool, float, float)> ctrl_signal_callback;
        std::thread thread_detection;
        cv::Point3f CalculateCoordinate(const cv::Point2f vertex[]);
        std::pair<float, float> CalculateAngle(const cv::Point3f &target);
        std::string camera_name;
        bool DetectArmor(cv::Mat &img, cv::Point3f &target);
        void Detection();
    public:
        Detector() = delete;
        Detector(std::string camera, float intrinsic_matrix[], float distortion_coeffs[], std::function<bool(bool, float, float)> callback);
        void StartDetection();
        void Spin();
        void StopDetection();
};

#endif
