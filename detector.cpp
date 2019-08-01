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
#include "detector.h"
#include <iostream>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif
using namespace std;
using namespace cv;

extern int verbose;
extern bool show_output, show_fps, show_img;

enum EnemyColor
{
    RED = 0,
    BLUE
} enemy = RED;

Point3f Detector::CalculateCoordinate(const Point2f vertex[])
{
    Mat rvec, tvec;
    vector<Point2f> ArmorVertex3D;
    for(int i = 0; i < 4; i++) ArmorVertex3D.push_back(vertex[i]);
    solvePnP(ArmorVertex2D, ArmorVertex3D, intrinsic, distortion, rvec, tvec);
    return Point3f(tvec);
}

pair<float, float> Detector::CalculateAngle(const Point3f &target)
{
    float yaw = atan2(target.x, target.z) / M_PI * 180;
    float pitch = atan2(target.y, sqrt(target.x*target.x + target.z*target.z)) / M_PI * 180;
    return make_pair(yaw, pitch);
}

Detector::Detector(std::string camera, float intrinsic_matrix[], float distortion_coeffs[], std::function<bool(bool, float, float)> callback)
{
    if(!cap.open(camera, CAP_V4L2)) throw runtime_error("cannot open camera");
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(CAP_PROP_FPS, 60);
    ArmorVertex2D.emplace_back(cv::Point3f(-120.0f/2, 200.0f+ 60.0f/2,  0.0));
    ArmorVertex2D.emplace_back(cv::Point3f(-120.0f/2, 200.0f-60.0f/2, 0.0));
    ArmorVertex2D.emplace_back(cv::Point3f(120.0f/2,  200.0f-60.0f/2, 0.0));
    ArmorVertex2D.emplace_back(cv::Point3f(120.0f/2,  200.0f+60.0f/2,  0.0));
    intrinsic = Mat(3, 3, CV_32F, intrinsic_matrix);
    distortion = Mat(1, 5, CV_32F, distortion_coeffs);
    ctrl_signal_callback = callback;
    running = false;
}

struct SingleLightbar
{
    // 1 2
    // 0 3
    Point2f center, vertex[4];
    float angle, breadth, length;
    void CalculateVertex()
    {
        const float theta = angle * CV_PI / 180;
        const float a = cos(theta) * 0.5f, b = sin(theta) * 0.5f;
        vertex[0].x = center.x - a * length - b * breadth;
        vertex[0].y = center.y + b * length - a * breadth;
        vertex[1].x = center.x + a * length - b * breadth;
        vertex[1].y = center.y - b * length - a * breadth;
        vertex[2].x = 2 * center.x - vertex[0].x;
        vertex[2].y = 2 * center.y - vertex[0].y;
        vertex[3].x = 2 * center.x - vertex[1].x;
        vertex[3].y = 2 * center.y - vertex[1].y;
    }
    SingleLightbar() : center(Point2f(0.0f, 0.0f)), angle(0.0f), breadth(0.0f), length(0.0f) { CalculateVertex(); }
    SingleLightbar(const Point2f &_center, const float &_angle, const float &_breadth, const float &_length) :
        center(_center), angle(_angle), breadth(_breadth), length(_length) { CalculateVertex(); }
    SingleLightbar(const RotatedRect &rect)
    {
        center = rect.center;
        if(rect.size.width < rect.size.height)
        {
            breadth = rect.size.width;
            length = rect.size.height;
            angle = 90.0f - rect.angle;
        }
        else
        {
            breadth = rect.size.height;
            length = rect.size.width;
            angle = -rect.angle;
        }
        CalculateVertex();
    }
    SingleLightbar Expand()
    {
        return SingleLightbar(center, angle, breadth * 3.0f, length * 1.5f);
    }
    RotatedRect toRotatedRect()
    {
        return RotatedRect(vertex[0], vertex[1], vertex[2]);
    }
};

struct ArmorPlate
{
    Point2f vertex[4];
    bool dq;
    double credibility, preferability;
    bool operator < (const ArmorPlate &rhs) const
    {
        return preferability > rhs.preferability;
    }
};

Mat Mask(Size size, Point2f p1, Point2f p2, Point2f p3, Point2f p4)
{
    Mat mask = Mat::zeros(size, CV_8UC1);
    Point pts[4];
    pts[0].x = p1.x; pts[0].y = p1.y;
    pts[1].x = p2.x; pts[1].y = p2.y;
    pts[2].x = p3.x; pts[2].y = p3.y;
    pts[3].x = p4.x; pts[3].y = p4.y;
    fillConvexPoly(mask, pts, 4, Scalar(255), 8, 0);
    return mask;
}

Mat Mask(Size size, Point2f vertex[4])
{
    return Mask(size, vertex[0], vertex[1], vertex[2], vertex[3]);
}

bool Detector::DetectArmor(Mat &img, Point3f &target)
{
#ifdef CUDA
    Mat hsv, gray;
    cuda::GpuMat img_gpu, hsv_gpu, gray_gpu;
    img_gpu.upload(img);
    cuda::cvtColor(img_gpu, hsv_gpu, COLOR_BGR2HSV);
    cuda::cvtColor(img_gpu, gray_gpu, COLOR_BGR2GRAY);
    hsv_gpu.download(hsv);
    gray_gpu.download(gray);
#else
    Mat hsv(img.rows, img.cols, CV_8UC3);
    Mat gray(img.rows, img.cols, CV_8UC1);
    cvtColor(img, hsv, COLOR_BGR2HSV);
    cvtColor(img, gray, COLOR_BGR2GRAY);
#endif
    vector<Mat> bgr;
    split(img, bgr);
    Mat color_sub;
    if(enemy == RED)
    {
        subtract(bgr[2], bgr[1], color_sub);
        inRange(color_sub, Scalar(50), Scalar(255), color_sub);
    }
    else
    {
        subtract(bgr[0], bgr[1], color_sub);
        inRange(color_sub, Scalar(90), Scalar(255), color_sub);
    }
    inRange(hsv, Scalar(0, 0, 235), Scalar(255, 255, 255), hsv);
    vector<vector<Point>> contours;
    findContours(hsv, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    vector<SingleLightbar> lights;
    vector<ArmorPlate> armors;
    map<int, vector<int>> light_armor_mapping;
    for(auto &ct : contours)
    {
        if(contourArea(ct) < 5 || contourArea(ct) > 2000) continue;
        SingleLightbar light(minAreaRect(ct));
        if(abs(light.angle - 90.0f) > 30.f) continue;
        Mat mask, mat_mean, mat_stddev;
        RotatedRect rrect = minAreaRect(ct);
        rrect.size.width *= 2;
        rrect.size.height *= 2;
        Point2f vertex[4];
        rrect.points(vertex);
        mask = Mask(gray.size(), vertex[0], vertex[1], vertex[2], vertex[3]);
        meanStdDev(color_sub, mat_mean, mat_stddev, mask);
        if(mat_mean.at<double>(0, 0) < 0.1) continue;
        lights.push_back(light);
    }
    for(int i = 0; i < lights.size(); i++)
    {
        for(int j = i+1; j < lights.size(); j++)
        {
            if(abs(lights[i].angle - lights[j].angle) > 15.0f) continue;
            float average_angle = (lights[i].angle + lights[j].angle) / 2;
            SingleLightbar left, right;
            if(lights[i].center.x < lights[j].center.x)
            {
                left = lights[i];
                right = lights[j];
            }
            else
            {
                left = lights[j];
                right = lights[i];
            }
            float dx = right.center.x - left.center.x;
            float dy = right.center.y - left.center.y;
            float slope = (abs(dx) < 2.0f ? -90.0f : atan(-dy/dx) * 180 / CV_PI);
            if(abs(average_angle - slope - 90) > 30.0f) continue;
            if(min(left.vertex[0].y, left.vertex[3].y) < max(right.vertex[1].y, right.vertex[2].y)) continue;
            if(max(left.vertex[1].y, left.vertex[2].y) > min(right.vertex[0].y, right.vertex[3].y)) continue;
            float distance = sqrt(dx*dx + dy*dy);
            if(distance > 6*min(left.length, right.length)) continue;
            if(distance < min(left.length, right.length)) continue;

            Mat mask, mat_mean, mat_stddev;
            mask = Mask(gray.size(), left.vertex[3], left.vertex[2], right.vertex[1], right.vertex[0]);
            meanStdDev(gray, mat_mean, mat_stddev, mask);
            if(mat_mean.at<double>(0, 0) > 90.0 || mat_stddev.at<double>(0, 0) > 90.0) continue;

            ArmorPlate candidate;
            candidate.vertex[0] = left.vertex[3];
            candidate.vertex[1] = left.vertex[2];
            candidate.vertex[2] = right.vertex[1];
            candidate.vertex[3] = right.vertex[0];
            candidate.dq = false;
            double parallelity = 1.0 - abs(lights[i].angle - lights[j].angle) / 15.0;
            double similarity = 1.0 / (max(lights[i].length, lights[j].length) / min(lights[i].length, lights[j].length));
            double orthogonality = abs(average_angle - slope) / 90.0;
            candidate.credibility = (parallelity + similarity + orthogonality) / 3.0;
            float yaw, pitch;
            tie(yaw, pitch) = CalculateAngle(CalculateCoordinate(candidate.vertex));
            double rotation = 1.0 - hypot(yaw, pitch) / 30.0;
            vector<Point> vertex, contour;
            vertex.emplace_back(candidate.vertex[0]);
            vertex.emplace_back(candidate.vertex[1]);
            vertex.emplace_back(candidate.vertex[2]);
            vertex.emplace_back(candidate.vertex[3]);
            convexHull(vertex, contour);
            double area = contourArea(contour) / 3000.0;
            candidate.preferability = (rotation + area) / 2.0;
            if(light_armor_mapping.find(i) == light_armor_mapping.end()) light_armor_mapping[i] = vector<int>();
            if(light_armor_mapping.find(j) == light_armor_mapping.end()) light_armor_mapping[j] = vector<int>();
            light_armor_mapping[i].emplace_back(armors.size());
            light_armor_mapping[j].emplace_back(armors.size());
            armors.push_back(candidate);
        }
    }

    for(auto it = light_armor_mapping.begin(); it != light_armor_mapping.end(); ++it)
    {
        if(it->second.size() > 1)
        {
            double maxv = armors[it->second[0]].credibility;
            int maxp = 0, i;
            for(int i = 1; i < it->second.size(); i++)
            {
                if(armors[it->second[i]].credibility > maxv)
                {
                    maxv = armors[it->second[i]].credibility;
                    maxp = i;
                }
            }
            for(int i = 0; i < it->second.size(); i++)
            {
                if(i != maxp) armors[it->second[i]].dq = true;
            }
        }
    }

    if(armors.empty()) return false;
    sort(armors.begin(), armors.end());
    int i = 0;
    for(; i < armors.size() && armors[i].dq; i++);
    if(i == armors.size()) return false;
    const ArmorPlate &armor = armors[i];
    Mat rvec, tvec;
    for(int i = 0; i < armors.size(); i++)
    {
        for(int j = 0; j < 4; j++)
            line(img, armors[i].vertex[j], armors[i].vertex[(j+1)%4], armors[i].dq ? Scalar(0, 0, 255) : Scalar(0, 255, 0), 3);
    }
    vector<Point2f> ArmorVertex3D;
    for(int i = 0; i < 4; i++) ArmorVertex3D.push_back(armor.vertex[i]);
    for(int i = 0; i < 4; i++) line(img, armor.vertex[i], armor.vertex[(i+1)%4], Scalar(255, 0, 0), 3);
    solvePnP(ArmorVertex2D, ArmorVertex3D, intrinsic, distortion, rvec, tvec);
    target = Point3f(tvec);
    return true;
}

void Detector::Detection()
{
    Mat img;
    Point3f target;
    float yaw, pitch;
    int frame_count = 0;
    chrono::high_resolution_clock::time_point last_frame_count = chrono::high_resolution_clock::now();
    while(running && cap.read(img))
    {
        if(DetectArmor(img, target))
        {
            tie(yaw, pitch) = CalculateAngle(target);
            if(show_output) cout << "(" << yaw << "," << pitch << ")" << endl;
            if(!ctrl_signal_callback(true, yaw, pitch)) running = false;
        }
        else
        {
            if(show_output) cout << "target not found" << endl;
            if(!ctrl_signal_callback(false, 0.0f, 0.0f)) running = false;
        }
        if(show_fps)
        {
            ++frame_count;
            if(chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - last_frame_count).count() > 1000)
            {
                cout << "FPS = " << frame_count << endl;
                frame_count = 0;
                last_frame_count = chrono::high_resolution_clock::now();
            }
        }
        if(show_img)
        {
            imshow("Detector", img);
            if(waitKey(1) == 27) running = false;
        }
    }
    running = false;
}

void Detector::StartDetection()
{
    running = true;
    thread_detection = thread(&Detector::Detection, this);
}

void Detector::Spin()
{
    while(running) this_thread::sleep_for(chrono::milliseconds(10));
}

void Detector::StopDetection()
{
    running = false;
    if(thread_detection.joinable()) thread_detection.join();
}
