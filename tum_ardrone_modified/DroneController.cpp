 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
 
 
 
#include "DroneController.h"
#include "gvars3/instances.h"
#include "../HelperFunctions.h"
#include "ControlNode.h"

DroneController::DroneController(void)
{
	target = DronePosition(TooN::makeVector(0.0,0.0,0.0),0.0);
	targetValid = false;
	last_err[2] = 0;
	lastTimeStamp = 0;

	hoverCommand.gaz = hoverCommand.pitch = hoverCommand.roll = hoverCommand.yaw = 0;

	node = NULL;
}


DroneController::~DroneController(void)
{
}

//Garantindo valor do ângulo entre -180 e +180
double angleFromTo2(double angle, double min, double sup)
{
	while(angle < min) angle += 360;
	while(angle >=  sup) angle -= 360;
	return angle;
}

//UPDATE
// generates and sends a new control command to the drone, 
// based on the currently active command and the drone's position.
ControlCommand DroneController::update(tum_ardrone::filter_stateConstPtr state)
{	
	//drone's position
	TooN::Vector<3> pose = TooN::makeVector(state->x, state->y, state->z);
	double yaw = state->yaw;
	TooN::Vector<4> speeds = TooN::makeVector(state->dx, state->dy, state->dz, state->dyaw);
	ptamIsGood = state->ptamState == state->PTAM_BEST || state->ptamState == state->PTAM_GOOD || state->ptamState == state->PTAM_TOOKKF;
	scaleAccuracy = state->scaleAccuracy;

	// ******load's speed
	TooN::Vector<2> load_vel = TooN::makeVector(state->dx_load, state->dy_load); 

	// calculate (new) errors.
	TooN::Vector<4> new_err = TooN::makeVector(
		target.pos[0] - pose[0],
		target.pos[1] - pose[1],
		target.pos[2] - pose[2],
		target.yaw - yaw
		);

	// ******Calculate load (new) errors.
	TooN::Vector<2> load_new_err = TooN::makeVector(
		0 - load_vel[0],
		0 - load_vel[1]
		);

	// yaw error needs special attention, it can always be pushed in between 180 and -180.
	// this does not affect speeds and makes the drone always take the quickest rotation side.
	new_err[3] = angleFromTo2(new_err[3],-180,180);	
	TooN::Vector<4> d_err = TooN::makeVector(-speeds[0], -speeds[1], -speeds[2], -speeds[3]);

	if(targetValid)
		//calcControl(new_err, d_err, yaw); 
		calcControl(new_err, d_err, yaw, load_new_err); // ********* Needed changing to add load_new_err
	else
	{
		lastSentControl = hoverCommand;
		ROS_WARN("Warning: no valid target, sending hover.");
	}

	last_err = new_err;
	last_load_err = load_new_err; // *************

	return lastSentControl;
}


void DroneController::setTarget(DronePosition newTarget)
{
	target = newTarget;
	target.yaw = angleFromTo2(target.yaw,-180,180);
	targetSetAtClock = getMS()/1000.0;
	targetNew = TooN::makeVector(1.0,1.0,1.0,1.0);
	targetValid = true;
	last_err = i_term = TooN::makeVector(0,0,0,0);

	char buf[200];
	snprintf(buf,200,"New Target: xyz = %.3f, %.3f, %.3f,  yaw=%.3f", target.pos[0],target.pos[1],target.pos[2],target.yaw);
	ROS_INFO(buf);

	if(node != NULL)
		node->publishCommand(std::string("u l ") + buf);
}


DronePosition DroneController::getCurrentTarget()
{
	return target;
}

void DroneController::clearTarget()
{
	targetValid = false;
}

void i_term_increase(double& i_term, double new_err, double cap)
{
	if(new_err < 0 && i_term > 0)
		i_term = std::max(0.0, i_term + 2.5 * new_err);
	else if(new_err > 0 && i_term < 0)
		i_term = std::min(0.0, i_term + 2.5 * new_err);
	else
		i_term += new_err;

	if(i_term > cap) i_term =  cap;
	if(i_term < -cap) i_term =  -cap;
}

//CONTROLADOR: MODIFICA SINAL LastSentControl (.roll, .pitch, .yaw, .gaz)
//void DroneController::calcControl(TooN::Vector<4> new_err, TooN::Vector<4> d_error, double yaw)
void DroneController::calcControl(TooN::Vector<4> new_err, TooN::Vector<4> d_error, double yaw, TooN::Vector<2> load_new_error)
{
	float agr = agressiveness;
	if(!ptamIsGood) agr *= 0.75;
	agr *= scaleAccuracy;

	/*TooN::Vector<4> d_term = new_err - last_err;	d-term:just differentiate */
	
	// Termo diferencial (d_term) = d_error = erro das velocidades (dx, dy, dz, dyaw)	 
	TooN::Vector<4> d_term = d_error;
	
	// Termo proporcional (p_term) = new_err = erro das posicoes (x, y, z, yaw)
	TooN::Vector<4> p_term = new_err;

	// *********** Termo proporcional (p_term_load) = load_new_err = erro das velocidades (dx_load, dy_load)
	TooN::Vector<2> p_term_load = load_new_error;

	// rotate error to drone coordinate system (invert pitch) **********????????????????
	//x and y change, z and yaw stay the same
	double yawRad = yaw * 2 * 3.141592 / 360;	
	d_term[0] = cos(yawRad)*d_error[0] - sin(yawRad)*d_error[1]; 	//x
	d_term[1] = - sin(yawRad)*d_error[0] - cos(yawRad)*d_error[1]; 	//y
	d_term[2] = d_term[2]; 						//z
	d_term[3] = d_term[3]; 						//yaw

	p_term[0] = cos(yawRad)*new_err[0] - sin(yawRad)*new_err[1];	//x
	p_term[1] = - sin(yawRad)*new_err[0] - cos(yawRad)*new_err[1];	//y
	p_term[2] = p_term[2];						//z
	p_term[3] = p_term[3]; 						//yaw

	// integrate & cap
	//Termo integral (i_term) = i_term + (new_err)*sec
	double sec = getMS()/1000.0 - lastTimeStamp; lastTimeStamp = getMS()/1000.0;
	i_term_increase(i_term[2],new_err[2] * sec, 0.2f / Ki_gaz); 		//z
	i_term_increase(i_term[1],new_err[1] * sec, 0.1f / Ki_rp+(1e-10)); 	//y
	i_term_increase(i_term[0],new_err[0] * sec, 0.1f / Ki_rp+(1e-10)); 	//x
	i_term_increase(i_term_load[1],load_new_error[1] * sec, 0.1f / Ki_rp_load+(1e-10)); 	//**********dy_load
	i_term_increase(i_term_load[0],load_new_error[0] * sec, 0.1f / Ki_rp_load+(1e-10)); 	//**********dx_load	

	// kill integral term when first crossing target		**********????????????????
	// that is, thargetNew is set, it was set at least 100ms ago, and err changed sign.
	for(int i=0;i<4;i++)
		if(targetNew[i] > 0.5 && getMS()/1000.0 - targetSetAtClock > 0.1 && last_err[i] * new_err[i] < 0)
		{
			i_term[i] = 0; targetNew[i] = 0;
		}

	//>>>>>>> YAW (only PD control)
	lastSentControl.yaw = Kp_yaw * p_term[3] + Kd_yaw * d_term[3];	// yaw can be translated directly
	/*CLIP*/ lastSentControl.yaw = std::min(max_yaw,std::max(-max_yaw,(double)(lastSentControl.yaw*agr)));

	//>>>>>>> RP (roll x, pitch y - PID control)
	// calculate signals only based on d and p: ?????????????

	double cX_p_load = Kp_rp_load * p_term_load[0];
	double cY_p_load = Kp_rp_load * p_term_load[1];

	double cX_i_load = Ki_rp_load * i_term_load[0];
	double cY_i_load = Ki_rp_load * i_term_load[1];

	if (UseLoadControl == false) { //To disable load control (dynamically recunfigurable)
		cX_p_load = 0;
		cY_p_load = 0;
		cX_i_load = 0;
		cY_i_load = 0;	}

	double cX_p = Kp_rp * p_term[0] + cX_p_load; //***********
	double cY_p = Kp_rp * p_term[1] + cY_p_load; //***********

	double cX_i = Ki_rp * i_term[0] + cX_i_load; //***********
	double cY_i = Ki_rp * i_term[1] + cY_i_load; //***********

	double cX_d = Kd_rp * d_term[0];
	double cY_d = Kd_rp * d_term[1];



	lastSentControl.roll = cX_p + cX_d + cX_i;
	lastSentControl.pitch = cY_p + cY_d + cY_i;

	/*CLIP*/ lastSentControl.roll = std::min(max_rp,std::max(-max_rp,(double)(lastSentControl.roll*agr)));
	/*CLIP*/ lastSentControl.pitch = std::min(max_rp,std::max(-max_rp,(double)(lastSentControl.pitch*agr)));

	//>>>>>>> GAZ (z - PID control)
	double gazP = Kp_gaz * p_term[2];
	double gazD = Kd_gaz * d_term[2];
	double gazI = Ki_gaz * i_term[2];

	/*CLIP*/lastSentControl.gaz = std::min(max_gaz_rise/rise_fac,std::max(max_gaz_drop, gazP + gazD + gazI));
	if(lastSentControl.gaz > 0) lastSentControl.gaz *= rise_fac;

	logInfo = TooN::makeVector( //**********????????????????
		Kp_rp * p_term[0], Kp_rp * p_term[1], gazP, Kp_yaw * p_term[3],
		Kd_rp * d_term[0], Kd_rp * d_term[1], gazD, Kd_yaw * d_term[3],
		Ki_rp * i_term[0], Ki_rp * i_term[1], gazI, Ki_yaw * i_term[3],
		lastSentControl.roll, lastSentControl.pitch, lastSentControl.gaz, lastSentControl.yaw,
		lastSentControl.roll, lastSentControl.pitch, lastSentControl.gaz, lastSentControl.yaw,
		new_err[0],new_err[1],new_err[2],new_err[3],
		target.pos[0],target.pos[1],target.pos[2],target.yaw
		);
}

TooN::Vector<4> DroneController::getLastErr()
{
	return last_err;
}
ControlCommand DroneController::getLastControl()
{
	return lastSentControl;
}
