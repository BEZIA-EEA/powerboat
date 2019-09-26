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

    see explanation of lift and drag explained here: https://en.wikipedia.org/wiki/Forces_on_sails

    To-Do: add heel handling by calculating lateral force from wind vs gravity force from heel to arrive at roll rate or acceleration
*/

#include "SIM_Powerboat.h"
#include <AP_Math/AP_Math.h>
#include <string.h>
#include <stdio.h>


namespace SITL {

Powerboat::Powerboat(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
	max_speed(3),
    max_accel(1),
    steering_angle_max(M_PI/4)
{
}

// return forward acceleration in m/s/s
float Powerboat::get_forw_accel(float X, float u, float v,  float r) const
{
    return r*v+(1/m)*X;
}


// return lateral acceleration in m/s/s
float Powerboat::get_lat_accel(float Y, float u, float v, float r) const
{
    return -u*r+(1/m)*Y; 
}

// return yaw accel in rad/s/s
float Powerboat::get_yaw_accel(float N, float u, float v,  float r) const
{
    return (1/Iz)*N; 
}


/*
  update the Powerboat simulation by one time step
 */
void Powerboat::update(const struct sitl_input &input)
{

    // in Powerboats the steering controls the rudder, the throttle controls the propelsor
    float throttle = 5 * 2*((input.servos[2]-1000)/1000.0f - 0.5f);

    // rudder angle (-45° to 45°)
    float delta = steering_angle_max * 2*((input.servos[0]-1000)/1000.0f - 0.5f);
	

    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    // speed along x axis, u
    float u = velocity_body.x;
	
    // speed along y axis, v
    float v = velocity_body.y;
	
    // yaw rate
    float r= gyro.z; //radians/s
	
		
	// propeller Force and torc 
	float n = throttle * max_accel;
	float Xp = C1*n*n;
	float Yp = 0;
	float Np = 0;
	
	// hull force and torc 
	float Xh =-mx*u+a1*u+a2*(u*u)+X_vv*(v*v)+(X_vr-my)*r*v+X_rr*(r*r)+X_vvvv*(v*v*v*v) ;
	float Yh =-my*v+Y_v*v+(Y_r-mx)*r+Y_vvv*(v*v*v)+Y_vvr*(v*v*r)+Y_vrr*(v*r*r)+Y_rrr*(r*r*r);
	float Nh =-Iz*r+N_v*v+N_r*r+N_vvv*(v*v*v)+N_vvr*(v*v*r)+N_vrr*(v*r*r)+N_rrr*(r*r*r);
	
	// rudder force and torc 
	float Fn= C2*(u*u+v*v);
	float Xr= -fabsf(Fn*sin(delta)*sin(delta));
	float Yr= 0.5*Fn*sin(2*delta);
	float Nr= xr*Yr;


    //force along X body
    float X = Xp+Xh+Xr;

    //force along Y body
    float Y = Yp+Yh+Yr;
	
    //Torc along Z body
    float N = Np+Nh+Nr;
	

    // yaw accel in radian
    float yaw_accel = get_yaw_accel(N,u,v,r);

    // update angular velocity
    r += yaw_accel * delta_time;
    
    gyro = Vector3f(0,0,r);


    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // accel in body frame due to motor
    accel_body = Vector3f(0, 0, 0);

    // add in accel due to direction change
    accel_body.x = get_forw_accel(X,u,v,r);// du/dt

    accel_body.y = get_lat_accel(Y,u,v,r);// dv/dt
	
    // now in earth frame
    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0, 0, GRAVITY_MSS);

    // we are on the ground, so our vertical accel is zero
    accel_earth.z = 0;

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    position += velocity_ef * delta_time;

    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
