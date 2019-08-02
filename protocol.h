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
#ifndef PROTOCOL_H_
#define PROTOCOL_H_

class Transmitter
{
    private:
        int serial_fd;
    public:
        Transmitter() = delete;
        Transmitter(const char *serial_device);
        ~Transmitter();
#ifdef SENTRY
        bool TransmitGimbalAngle(bool gimbal_front, const float yaw, const float pitch, bool chassis_rear_left, bool chassis_rear_right);
        bool TransmitShootCmd(bool shoot);
#else
        bool TransmitGimbalAngle(const float yaw, const float pitch);
#endif
};

#endif //PROTOCOL_H_
