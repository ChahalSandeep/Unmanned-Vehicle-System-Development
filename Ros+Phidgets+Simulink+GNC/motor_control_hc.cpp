/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets motor control HC
 *  Copyright (c) 2010, Bob Mottram
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <phidget21.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "phidgets/motor_params.h"

//modified
#include <std_msgs/String.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"
#include "phidgets/encoder_params.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

//modified

// handle
CPhidgetMotorControlHandle phid;

// motor controller state publisher
ros::Publisher motors_pub;


//modified
// encoder state publisher
ros::Publisher encoder_pub;

ros::Publisher encoder_count_pub;

ros::Publisher analog_input1_pub;
ros::Publisher analog_input2_pub;

ros::Publisher digital_input1_pub;
ros::Publisher digital_input2_pub;

ros::Publisher motor_velocity_pub;
//modified


float speed = 20;
float acceleration = 20;
bool x_forward = true;
bool invert_rotation = false;
bool invert_forward = false;
double rotation_offset = 0;

nav_msgs::Odometry odom;
double current_linear_velocity = 0;
double current_angular_velocity = 0;
double linear_velocity_proportional = 0.1;
double linear_velocity_integral = 0.0;
double linear_velocity_derivative = 0.0;
double angular_velocity_proportional = 0.1;
double angular_velocity_integral = 0.0;
double angular_velocity_derivative = 0.0;
double max_angular_velocity = 0.1;
double linear_deadband = 0.02;
double angular_deadband = 0.02;
double max_angular_error = 10*3.1415927/180.0;
double max_velocity_error = 0.05;
double max_angular_accel = 0.5;
double max_linear_accel = 0.3;
double ITerm[2];
double last_v=0,last_angv=0;

bool odometry_active = false;



ros::Time last_velocity_command;

bool motors_active = false;
bool initialised = false;



int AttachHandler(CPhidgetHandle phid, void *userptr)
{
    int serial_number;
    const char *name;

    CPhidget_getDeviceName (phid, &name);
    CPhidget_getSerialNumber(phid, &serial_number);
    ROS_INFO("%s Serial number %d attached!",
			 name, serial_number);

    return 0;
}

int DetachHandler(CPhidgetHandle phid, void *userptr)
{
    int serial_number;
    const char *name;

    CPhidget_getDeviceName (phid, &name);
    CPhidget_getSerialNumber(phid, &serial_number);
    ROS_INFO("%s Serial number %d detached!",
			 name, serial_number);

    return 0;
}

int ErrorHandler(CPhidgetHandle phid, void *userptr,
				 int ErrorCode, const char *Description)
{
    ROS_INFO("Error handled. %d - %s", ErrorCode, Description);
    return 0;
}

int InputChangeHandler(CPhidgetMotorControlHandle MC,
					   void *usrptr, int Index, int State)
{
    phidgets::motor_params m;
    m.index = Index;
    m.value_type = 1;
    m.value = (float)State;

    int Digital1,Digital2;
    std_msgs::Int16 Digital_Input1,Digital_Input2;

	CPhidgetMotorControl_getInputState(MC, 0, &Digital1);
	CPhidgetMotorControl_getInputState(MC, 1, &Digital2);
    
    Digital_Input1.data = Digital1;
    Digital_Input2.data = Digital2;

    if (initialised) 
	{
	motors_pub.publish(m);
	digital_input1_pub.publish(Digital_Input1);
	digital_input2_pub.publish(Digital_Input2);
	}
    //ROS_INFO("Motor input %d Inputs %d", Index, State);
    return 0;
}

int VelocityChangeHandler(CPhidgetMotorControlHandle MC,
						  void *usrptr, int Index, double Value)
{
    
    phidgets::motor_params m;
    m.index = Index;
    m.value_type = 2;
    m.value = (float)Value;

    
    if (initialised) 
	{
	motors_pub.publish(m);
	
	}
    //ROS_INFO("Motor %d Velocity %.2f", Index, (float)Value);
    return 0;
}

int CurrentChangeHandler(CPhidgetMotorControlHandle MC,
						 void *usrptr, int Index, double posChange)
{
    phidgets::motor_params m;
    m.index = Index;
    m.value_type = 3;
    m.value = (float)posChange;
    if (initialised) motors_pub.publish(m);
    //ROS_INFO("Motor %d Current %.2f", Index, (float)Value);
    return 0;
}


int EncoderChangeHandler(CPhidgetMotorControlHandle MC,
						 void *usrptr, int Index, int time, int RelativePosition)
{
	int Position;
	std_msgs::Int16 etick;
	CPhidgetMotorControl_getEncoderPosition(MC, Index, &Position);

	phidgets::encoder_params e;
	e.index = Index;
    	e.count = Position;
    	e.count_change = RelativePosition;
	e.time = time;

	etick.data = Position;
	

	if (initialised) encoder_pub.publish(e);
	encoder_count_pub.publish(etick);
   	ROS_INFO("Encoder %d Count %d", Index, Position);	
    	return 0;
}

int SensorUpdateHandler(CPhidgetMotorControlHandle MC,
						 void *usrptr, int Index, int SensorValue)
{
	int Analog1,Analog2;
	std_msgs::Int16 AnalogInput1, AnalogInput2;	
	CPhidgetMotorControl_getSensorValue(MC, 0, &Analog1);
	CPhidgetMotorControl_getSensorValue(MC, 1, &Analog2);

	AnalogInput1.data = Analog1;
	AnalogInput2.data = Analog2;
	
	double Value;
	std_msgs::Float64 Velocity;
	CPhidgetMotorControl_getVelocity(MC, Index, &Value);
   	Velocity.data = (float)Value;
	if (initialised) 
	{
	analog_input1_pub.publish(AnalogInput1);
	analog_input2_pub.publish(AnalogInput2);
	motor_velocity_pub.publish(Velocity);
	}
   	//ROS_INFO("Sensor %d Value %d", Index, Value);	//Sensor value range from 0-1000
    	return 0;
}


int display_properties(CPhidgetMotorControlHandle phid)
{
    int serial_number, version, num_motors, num_Digital_inputs, num_Analog_inputs, count;
    const char* ptr;

    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid,
							 &serial_number);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

    CPhidgetMotorControl_getInputCount(phid, &num_Digital_inputs);
    CPhidgetMotorControl_getMotorCount(phid, &num_motors);
    
    CPhidgetMotorControl_getSensorCount(phid, &num_Analog_inputs);
    CPhidgetMotorControl_getEncoderCount (phid, &count);

    ROS_INFO("%s", ptr);
    ROS_INFO("Serial Number: %d", serial_number);
    ROS_INFO("Version: %d", version);
    ROS_INFO("Number of motors %d", num_motors);
    ROS_INFO("Number of Digital inputs %d", num_Digital_inputs);

    ROS_INFO("Number of Analog inputs %d", num_Analog_inputs);
    ROS_INFO("# Encoder: %d", count);

    return 0;
}



bool attach(
			CPhidgetMotorControlHandle &phid,
			int serial_number)
{
    // create the object
    CPhidgetMotorControl_create(&phid);

    // Set the handlers to be run when the device is
	// plugged in or opened from software, unplugged
	// or closed from software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid,
								  AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid,
								  DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)phid,
								 ErrorHandler, NULL);

    // Registers a callback that will run if an input changes.
    // Requires the handle for the Phidget, the function
	// that will be called, and a arbitrary pointer that
	// will be supplied to the callback function (may be NULL).
    CPhidgetMotorControl_set_OnInputChange_Handler (phid,
													InputChangeHandler,
													NULL);

    // Registers a callback that will run if a motor changes.
    // Requires the handle for the Phidget, the function
	// that will be called, and a arbitrary pointer that
	// will be supplied to the callback function (may be NULL).
    CPhidgetMotorControl_set_OnVelocityChange_Handler (phid,
													   VelocityChangeHandler,
													   NULL);

    // Registers a callback that will run if the current
	// draw changes.
    // Requires the handle for the Phidget, the function
	// that will be called, and a arbitrary pointer that
	// will be supplied to the callback function (may be NULL).
    CPhidgetMotorControl_set_OnCurrentChange_Handler (phid,
													  CurrentChangeHandler,
													  NULL);

    CPhidgetMotorControl_set_OnEncoderPositionChange_Handler(phid, EncoderChangeHandler, NULL);

    CPhidgetMotorControl_set_OnSensorUpdate_Handler(phid, SensorUpdateHandler, NULL);
   
    //open the device for connections
    CPhidget_open((CPhidgetHandle)phid, serial_number);

    // get the program to wait for an motor control
	// device to be attached
    if (serial_number == -1) {
        ROS_INFO("Waiting for Motor Control Phidget " \
				 "to be attached....");
    }
    else {
        ROS_INFO("Waiting for Motor Control Phidget " \
				 "%d to be attached....", serial_number);
    }
    int result;
    if((result =
		CPhidget_waitForAttachment((CPhidgetHandle)phid,
								   10000)))
		{
			const char *err;
			CPhidget_getErrorDescription(result, &err);
			ROS_ERROR("Problem waiting for motor " \
					  "attachment: %s", err);
			return false;
		}
    else return true;
}

/*!
 * \brief disconnect the motor controller
 */
void disconnect(CPhidgetMotorControlHandle &phid)
{
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}




void encoderSetCallback(const std_msgs::Int16::ConstPtr& ptr)
{
    if (initialised) {
    	std_msgs::Int16 EncoderPosition = *ptr;
	
	int SetPosition;
	
	SetPosition = EncoderPosition.data;

	CPhidgetMotorControl_setEncoderPosition(phid, 0, SetPosition);
    }
}

/*!
 * \brief callback when a velocity command is received
 * \param ptr encoder parameters
 */
void velocityCommandCallback(const geometry_msgs::Twist::ConstPtr& ptr)
{
    if (initialised) {
        geometry_msgs::Twist m = *ptr;
        // convert Twist to motor velocities
     
        float x = m.linear.x;
       

        ros::Time current_time = ros::Time::now();
       
	double vel = x; //between 0 to 100, and minus
	double acc = 0.5;//acceleration to obtain desired velocity
                   
                    
            CPhidgetMotorControl_setVelocity (phid, 0, vel);
											  -
          
            CPhidgetMotorControl_setAcceleration (phid, 0, acc);
												  
         
        
        last_velocity_command = current_time;
        motors_active = true;
    }
}


	
void stop_motors()
{
    CPhidgetMotorControl_setVelocity (phid, 0, 0);
    
    motors_active = false;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "phidgets_motor_control");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    int serial_number = -1;
    nh.getParam("serial", serial_number);
    std::string name = "motorcontrol";
    nh.getParam("name", name);
   
  


    std::string topic_path = "phidgets/";
    nh.getParam("topic_path", topic_path);
    int timeout_sec = 2;
    nh.getParam("timeout", timeout_sec);
    
    int frequency = 30;
    nh.getParam("frequency", frequency);

   

    if (attach(phid, serial_number)) {
		display_properties(phid);

        const int buffer_length = 100;        
        std::string topic_name = topic_path + name;
        std::string service_name = name;
        if (serial_number > -1) {
            char ser[10];
            sprintf(ser,"%d", serial_number);
            topic_name += "/";
            topic_name += ser;
            service_name += "/";
            service_name += ser;
        }
        motors_pub =
			n.advertise<phidgets::motor_params>(topic_name,
												buffer_length);

//modified
	encoder_pub =
			n.advertise<phidgets::encoder_params>("encoderParams",
												  buffer_length);

	// Publish Encoder Thicks

	encoder_count_pub = n.advertise<std_msgs::Int16>("eTick",buffer_length);
 	
	//Analog Input Sensor Values

	analog_input1_pub = n.advertise<std_msgs::Int16>("AnalogInput1",buffer_length);
	analog_input2_pub = n.advertise<std_msgs::Int16>("AnalogInput2",buffer_length);

	//Digital Input States

	digital_input1_pub = n.advertise<std_msgs::Int16>("DigitalInput1",buffer_length);
	digital_input2_pub = n.advertise<std_msgs::Int16>("DigitalInput2",buffer_length);

	//Motor Current Velocity
	motor_velocity_pub = n.advertise<std_msgs::Float64>("MotorVelocity",buffer_length);
//modified



        // receive velocity commands
        ros::Subscriber command_velocity_sub =
			n.subscribe("cmd_vel", 0, velocityCommandCallback);

	// receive encoder reset command
        ros::Subscriber reset_eThick_sub =
			n.subscribe("SetEncoderPosition", buffer_length, encoderSetCallback);

        // subscribe to odometry
      //  ros::Subscriber odometry_sub =
	//		n.subscribe(odometry_topic, 0, odometryCallback);

        initialised = true;
        ros::Rate loop_rate(frequency);

        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();

            // SAFETY FEATURE
            // if a velocity command has not been received
			// for a period of time then stop the motors
            double time_since_last_command_sec =
				(ros::Time::now() -
				 last_velocity_command).toSec();
            if ((motors_active) &&
				(time_since_last_command_sec > timeout_sec)) {
                stop_motors();        
                ROS_WARN("No velocity command received - " \
						 "motors stopped");        
            }
        }

        disconnect(phid);
    }
    return 0;
}

