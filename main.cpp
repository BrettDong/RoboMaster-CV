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
Detector *detector;
Transmitter *transmitter;

void clean_up()
{
    if(detector) detector->StopDetection();
    if(transmitter) delete transmitter;
    delete detector;
    detector = nullptr;
    transmitter = nullptr;
}

void sig_handler(int sig)
{
    clean_up();
}

bool ctrl_signal_callback(bool detected, float yaw, float pitch)
{
    if(transmitter) return transmitter->TransmitGimbalAngle(yaw, pitch);
    else return true;
}

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
    detector->Spin();
    clean_up();
    return 0;
}
