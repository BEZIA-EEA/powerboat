/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Powerboat simulator class
*/

#pragma once
#include "SIM_Aircraft.h"
namespace SITL {
/*
  a Powerboat simulator
 */
class Powerboat : public Aircraft {
public:
    Powerboat(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Powerboat(home_str, frame_str);
    }

private:
	
	float steering_angle_max;   // vehicle steering mechanism's max angle in degrees
	float max_speed;
    float max_accel;
	/*
    const float m = 3.50;
	const float mx = 0.82;
    const float my = 0.93;
    const float Iz = 0.05647;
	const float iz = 0.08647;
	const float C1 = 1;
	const float C2 = 0.018;
	const float xr = 0.75;
    */
	
	
    const float m = 50;
	const float mx = 5.5;
    const float my = 6;
    const float Iz = 0.55;
	const float C1 = 1;
	const float C2 = 3;
	const float xr = 0.5;
	
	const float a1 = -1;
	const float a2 = -1;

	const float N_v = 1.332939677;
	const float N_r = -10.48233677;
	const float Y_v = -2.1580152;
	const float Y_r = -1.617822171;
	const float X_vv = 0.277948314;
	const float X_vr = 1.080967171;
	const float Y_rrr = -0.557315629;
	const float Y_vvv = 1.079399514;
	const float N_vvr = 1.162269114;
	const float N_vrr = 0.376589086;

    const float X_vvvv = 0;
	const float X_rr = 0;
	const float Y_vvr =0;
	const float Y_vrr =0;
	const float N_vvv =0;
	const float N_rrr =0;

	
	/*
    const float m = 15.50;
	const float mx = 2.0;
    const float my = 2.4;
    const float Iz = 1.8;
	const float iz = 1.9;
	const float C1 = 1;
	const float C2 = 0.8;
	const float xr = 1;
    */
	
	
    // return forward acceleration in m/s/s given a steering input (in the range -1 to +1) and speed in m/s 
    float get_forw_accel(float X, float u, float v,  float r) const;

    // return lateral acceleration in m/s/s given a steering input (in the range -1 to +1) and speed in m/s
    float get_lat_accel(float Y, float u, float v, float r) const;

    // return yaw rate in deg/sec given a steering input (in the range -1 to +1) and speed in m/s
    float get_yaw_accel(float N, float u, float v,  float r) const;

  
	
};

} // namespace SITL
