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
#include <cstdlib>
#include <cstring>
#include <string>
#include <cstdio>
#include <chrono>
#include <signal.h>
#include "constraint_set.h"
#include "protocol.h"
using namespace std;
using namespace cv;

void sig_handler(int sig)
{
    protocol::Disconnect();
    exit(0);
}

void write(cv::Mat &img, const char *str, const cv::Point &pt)
{
    cv::Size text_size = cv::getTextSize(str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, 0);
    cv::rectangle(img, pt, pt + cv::Point(text_size.width, -text_size.height), cv::Scalar(0, 0, 0), CV_FILLED);
    cv::putText(img, str, pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, 8);
}

int main(int argc, char *argv[])
{
    int verbose = 0;
    string serial_device("/dev/ttyUSB0");
    if(argc > 1)
    {
        for(int i = 1; i < argc; i++)
        {
            if(strcmp(argv[i], "--verbose") == 0 && i+1 < argc)
            {
                verbose = argv[++i][0] - '0';
            }
            else if(strcmp(argv[i], "--serial") == 0 && i+1 < argc)
            {
                serial_device = argv[++i];
            }
            else if(strcmp(argv[i], "--help") == 0)
            {
                cout << "Available parameters: " << endl;
                cout << "  --verbose <verbose_level>" << endl;
                cout << "  Specify verbose level" << endl;
                cout << "    0 (default) - Only output I/O information on successful start-up" << endl;
                cout << "    1 - Show real-time recognition and transmission performance on a window" << endl;
                cout << endl;
                cout << "  --serial <serial_device>" << endl;
                cout << "  Specify serial device for control signal transmission" << endl;
                cout << "    /dev/ttyUSB0 (default)" << endl;
                return 0;
            }
        }
    }
    if(!protocol::Connect(serial_device.c_str()))
    {
        cerr << "Unable to connect " << serial_device << endl;
        return 1;
    }
    VideoCapture cap;
    Mat img, res;
    float intrinsic_matrix[] = { 1536.07f, 0.0f, 320.0f,
                                       0.0f, 1542.55f, 240.0f,
                                       0.0f, 0.0f, 1.0f };
    float distortion_coeffs[] = { 0.44686f, 15.5414f, -0.009048f, -0.009717f, -439.74f };
    constraint_set::InitializeConstraintSet(intrinsic_matrix, distortion_coeffs);
    Point3f target;
    if(!cap.open(0) || !cap.read(img))
    {
        cerr << "Unable to open camera" << endl;
        return 1;
    }
    char buf[100];
    bool running = true;
    float angle_amp = 1.0f;
    signal(SIGINT, sig_handler);
    signal(SIGKILL, sig_handler);
    cout << "INPUT:  CAMERA " << img.cols << "x" << img.rows << endl;
    cout << "OUTPUT: " << serial_device << endl;
    float yaw = 0.0f, pitch = 0.0f;
    bool send = true;
    while(running)
    {
        auto Tstart = chrono::system_clock::now();
        if(!cap.read(img))
            break;
        constraint_set::DetectArmor(img, target);
        if(send)
        {
            write(img, "[TX]", cv::Point(10, 40));
            protocol::Send(yaw * angle_amp, pitch * angle_amp);
        }
        sprintf(buf, "YAW=% 4.2fDEG PITCH=% 4.2fDEG", yaw, pitch);
        write(img, buf, cv::Point(10, 60));
        sprintf(buf, "AMP%fx YAW= % 4.2fDEG PITCH=% 4.2fDEG", angle_amp, yaw * angle_amp, pitch * angle_amp);
        write(img, buf, cv::Point(10, 80));
        if(verbose > 0)
        {
            auto Tend = chrono::system_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(Tend - Tstart);
            const float latency = static_cast<float>(duration.count());// * chrono::milliseconds::period::num / chrono::milliseconds::period::den;
            sprintf(buf, "LATENCY: % 4.2f MS | %d FPS", latency, static_cast<int>(1000.0f / latency));
            write(img, buf, cv::Point(5, 20));
            line(img, Point(img.cols/2 - 10, img.rows/2), Point(img.cols/2 + 10, img.rows/2), Scalar(255, 0, 0));
            line(img, Point(img.cols/2, img.rows/2 - 10), Point(img.cols/2, img.rows/2 + 10), Scalar(255, 0, 0));
            imshow("constraint_set", img);
            switch(waitKey(1))
            {
                case 27:
                    running = false;
                    break;
                case 'a':
                    yaw += 1.0f;
                    break;
                case 'd':
                    yaw -= 1.0f;
                    break;
                case 'w':
                    pitch += 1.0f;
                    break;
                case 's':
                    pitch -= 1.0f;
                    break;
                case 'f':
                    send = !send;
                    break;
            }
        }
    }
    protocol::Disconnect();
    return 0;
}
