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
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <string>
#include <cstdio>
#include <chrono>
#include <mutex>
#include <atomic>
#include <thread>
#include <signal.h>
#include <unistd.h>
#include "detector.h"
#include "protocol.h"
using namespace std;
using namespace cv;
bool show_output, show_fps, show_img;
#ifdef SENTRY
Detector *detector, *detector_chassis_left, *detector_chassis_right;
atomic<bool> chassis_left, chassis_right;
#else
Detector *detector;
#endif
Transmitter *transmitter;

void clean_up()
{
    if(detector) { detector->StopDetection(); delete detector; detector = nullptr; }
#ifdef SENTRY
    if(detector_chassis_left) { detector_chassis_left->StopDetection(); delete detector_chassis_left; detector_chassis_left = nullptr; }
    if(detector_chassis_right) { detector_chassis_right->StopDetection(); delete detector_chassis_right; detector_chassis_right = nullptr; }
#endif
    if(transmitter) { delete transmitter; transmitter = nullptr; }
}

void sig_handler(int sig)
{
    clean_up();
}

#ifdef SENTRY
bool ctrl_signal_callback(bool detected, float yaw, float pitch)
{
    if(transmitter) return transmitter->TransmitGimbalAngle(detected, yaw, pitch, chassis_left, chassis_right) && transmitter->TransmitShootCmd(detected && hypot(yaw, pitch) < 2.0f);
    else return true;
}
bool chassis_left_callback(bool detected, float, float) { chassis_left = detected; return true; }
bool chassis_right_callback(bool detected, float, float) { chassis_right = detected; return true; }
#else
bool ctrl_signal_callback(bool detected, float yaw, float pitch)
{
    if(transmitter) return transmitter->TransmitGimbalAngle(yaw, pitch);
    else return true;
}
#endif

int main(int argc, char *argv[])
{
    bool dummy = false;
    for(int i = 1; i < argc; i++)
    {
        if(strcmp(argv[i], "--dummy") == 0)
        {
            dummy = true;
        }
        else if(strcmp(argv[i], "--show-fps") == 0)
        {
            show_fps = true;
        }
        else if(strcmp(argv[i], "--show-output") == 0)
        {
            show_output = true;
        }
        else if(strcmp(argv[i], "--show-img") == 0)
        {
            show_img = true;
        }
        else
        {
            cout << "Unrecognized parameter " << argv[i] << endl;
            return 1;
        }
    }
    Mat img, res;
    float intrinsic_matrix[9];
    float distortion_coeffs[5];
    ifstream fin("camera.txt");
    if(!fin)
    {
        cout << "Camera calibration data not available" << endl;
        return 1;
    }
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
            fin >> intrinsic_matrix[i*3+j];
    }
    for(int i = 0; i < 5; i++)
        fin >> distortion_coeffs[i];
    fin.close();
    try
    {
        detector = new Detector("/dev/video0", intrinsic_matrix, distortion_coeffs, ctrl_signal_callback);
#ifdef SENTRY
        detector_chassis_left = new Detector("/dev/video1", intrinsic_matrix, distortion_coeffs, chassis_left_callback);
        detector_chassis_right = new Detector("/dev/video2", intrinsic_matrix, distortion_coeffs, chassis_right_callback);
#endif
    }
    catch(exception &e)
    {
        cout << "Detector initialization failed" << endl;
        cout << e.what() << endl;
        return 1;
    }
    if(!dummy)
    {
        try
        {
            transmitter = new Transmitter("/dev/serial_sdk");
        }
        catch(exception &e)
        {
            cout << "Unable to connect to STM32" << endl;
            cout << e.what() << endl;
            delete detector;
            return 1;
        }
    }
    signal(SIGINT, sig_handler);
    signal(SIGKILL, sig_handler);
    signal(SIGTERM, sig_handler);
    detector->StartDetection();
#ifdef SENTRY
    detector_chassis_left->StartDetection();
    detector_chassis_right->StartDetection();
#endif
    detector->Spin();
#ifdef SENTRY
    detector_chassis_left->Spin();
    detector_chassis_right->Spin();
#endif
    clean_up();
    return 0;
}
