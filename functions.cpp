/*
    Keyframe extraction with semantic graphs in assembly processes.
    Copyright (C) 2016-2025  Grigorios Piperagkas

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


void rotmat_to_euler(const Eigen::Matrix4f& rotmat, float&x, float& y, float& z,
                     float& orient_x, float& orient_y, float& orient_z) {
    Eigen::Matrix4f R = rotmat;
 
    orient_z = atan2(R(1, 0), R(0, 0));
    float a = sqrt(R(2, 1)*R(2, 1) + R(2, 2)*R(2, 2));
    orient_y = atan2(-R(2,0), a);
    orient_x = atan2(R(2,1), R(2,2));

    x = R(0,3);
    y = R(1,3);
    z = R(2,3);
}


Eigen::Matrix4f get_rotmat_from_yaw_pitch_roll(float x, float y, float z, float yaw, float pitch, float roll){

    float cos_yaw = cos(yaw);
    float sin_yaw = sin(yaw);
    float cos_pitch = cos(pitch);
    float sin_pitch = sin(pitch);
    float cos_roll = cos(roll);
    float sin_roll = sin(roll);

    Eigen::Matrix4f Rz;
    Rz << cos_yaw, -sin_yaw,  0, 0,
          sin_yaw,  cos_yaw,  0, 0,
          0,        0,        1, 0,
          0,        0,        0, 1;

    Eigen::Matrix4f Ry;
    Ry <<  cos_pitch,  0,  sin_pitch, 0,
          0,           1,  0,         0,
          -sin_pitch,  0,  cos_pitch, 0,
          0,           0,  0,         1;

    Eigen::Matrix4f Rx;
    Rx << 1,  0,          0,        0,
          0,  cos_roll,  -sin_roll, 0,
          0,  sin_roll,   cos_roll, 0,
          0,  0,          0,        1;

    Eigen::Matrix4f Res = (Rz * Ry * Rx);

    R(0,3) = x;
	R(1,3) = y;
	R(2,3) = z;
	

}
