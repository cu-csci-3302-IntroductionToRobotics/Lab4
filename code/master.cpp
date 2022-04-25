#include "master.h"

master::master(){
    // attach to robot interface
    robot.reset(new RobotInterface);
}

void master::Run(){

state_t state = state_t::follow_line;

double LIDAR_SENSOR_MAX_RANGE = 3; // Meters
double LIDAR_ANGLE_BINS = 21; // 21 Bins to cover the angular range of the lidar, centered at 10
double LIDAR_ANGLE_RANGE = 1.5708; // 90 degrees, 1.5708 radians

// These are your pose values that you will update by solving the odometry equations
double pose_x = 0.197;
double pose_y = 0.678;
double pose_theta = 0;

// ePuck Constants
double EPUCK_AXLE_DIAMETER = 0.053; // ePuck's wheels are 53mm apart.
double MAX_SPEED = 6.28;


////////// Part 1: Setup Data structures
//
// Create an empty list for your lidar sensor readings here,
// as well as an array that contains the angles of each ray 
// in radians. The total field of view is LIDAR_ANGLE_RANGE,
// and there are LIDAR_ANGLE_BINS. An easy way to generate the
// array that contains all the angles is to use linspace from
// the numpy package.



//////// End of Part 1 //////////

double vL = 0; // Left wheel velocity in rad/s
double vR = 0; // Right wheel velocity in rad/s

double EPUCK_MAX_WHEEL_SPEED;
double dsr;
double dsl;
double ds;

// vector to store ground sensor readings
vector<double> gsens;
 
// Main Control Loop:
while (robot->StepSim() != -1) {  
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                 Sensing                           //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Read ground sensors
    robot->getGroundSensors(gsens);

    // Read Lidar data        
    cv::Mat lidar_readings;
    robot->getLidarRangeImage(lidar_readings);

    ////////// Part 2: Turn world coordinates into map coordinates
    //
    // Come up with a way to turn the robot pose (in world coordinates)
    // into coordinates on the map. Draw a red dot using display.drawPixel()
    // where the robot moves.
    

    
    
    ////////// Part 3: Convert Lidar data into world coordinates
    //
    // Each Lidar reading has a distance rho and an angle alpha.
    // First compute the corresponding rx and ry of where the lidar
    // hits the object in the robot coordinate system. Then convert
    // rx and ry into world coordinates wx and wy. 
    // The arena is 1x1m2 and its origin is in the top left of the arena. 
    

    
    
    ////////// Part 4: Draw the obstacle and free space pixels on the map
 
    
          

    
 

    
    // DO NOT MODIFY THE FOLLOWING CODE
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                 Robot controller                  //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    switch (state)
    {
    case state_t::follow_line :
        if(gsens[1]<350 && gsens[0]>400 && gsens[2]>400){
            vL = MAX_SPEED*0.3;
            vR = MAX_SPEED*0.3;
        } else if(gsens[0]<500 && gsens[1]<500 && gsens[2]<500){
            vL = MAX_SPEED*0.3;
            vR = MAX_SPEED*0.3;
            // save currently displayed image to file
            robot->SaveDisplayImageToFile("map.png");
        } else if(gsens[2]<650){
            vL = MAX_SPEED*0.2;
            vR = -MAX_SPEED*0.05;
        } else if(gsens[0]<650){
            vL = -MAX_SPEED*0.05;
            vR = MAX_SPEED*0.2;
        }
        break;
    case state_t::stop :
        vL = 0.0;
        vR = 0.0;
        break;
    default:
        vL = 0.0;
        vR = 0.0;
        break;
    }

    robot->SetLeftMotorSpeed(vL);
    robot->SetRightMotorSpeed(vR);
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                    Odometry                       //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    EPUCK_MAX_WHEEL_SPEED = 0.11695*robot->GetTimeStep()/1000.0;
    dsr=vR/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED;
    dsl=vL/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED;
    ds=(dsr+dsl)/2.0;
    
    pose_y += ds*cos(pose_theta);
    pose_x += ds*sin(pose_theta);
    pose_theta += (dsr-dsl)/EPUCK_AXLE_DIAMETER;
}
}