#include<tum_ics_ur_robot_lli/Robot/RobotArmConstrained.h>
#include<tum_ics_ur10_controller_tutorial/SimpleEffortControl.h>

#include<QApplication>

#include<visualization_msgs/Marker.h>

using namespace Eigen;

int main(int argc, char **argv)
{

    QApplication a(argc,argv);

    ros::init(argc,argv,"testRobotArmClass",ros::init_options::AnonymousName);


    QString configFilePath=argv[1];

    ROS_INFO_STREAM("Config File: "<<configFilePath.toStdString().c_str());

    tum_ics_ur_robot_lli::Robot::RobotArmConstrained robot(configFilePath);


    //starts robotArm communication and the thread
    //inits Kinematic Model, Dynamic Model
    if(!robot.init())
    {
        return -1;
    }

    tum_ics_ur_robot_lli::RobotControllers::SimpleEffortControl simpleEffortControl;



    //The control must be connected to the robot after the init()-->The dynamic model needs to
    //be initialized!
    //also calls control.init(), e.g. load ctrl gains
    if(!robot.add(&simpleEffortControl))
    {
        return -1;
    }


    simpleEffortControl.setQHome(robot.qHome());
    simpleEffortControl.setQPark(robot.qPark());


    robot.start();

    ROS_INFO_STREAM("Start main thread");

//    ros::Rate r(30);

//    while((ros::ok())&&(robot.isRunning()))
//    {
//        ros::spinOnce();
//        r.sleep();
//    }

    ros::spin();

    ROS_INFO_STREAM("main: Stoping RobotArm()");
    robot.stop(); //stops robotArm thread




    ROS_INFO_STREAM("main: Stopped!!!");

}








//    QString scriptFilePath="/home/dean/indigo_ws_iros_flo/src/tom_robot/tom_configs/Arms/parameters/Scripts/progTOM_right";

//    ur10_robot_lli::CommInterface commInterface(computerIp, commandPort, trajPort,robotIp, scriptFilePath, robotScriptPort);


//    usleep(20*1E6);


//    //commInterface.stop();

//    commInterface.setFinishMState();


//    ur10_robot_lli::ScriptLoader scriptLoader(robotIp,
//    "/home/dean/indigo_ws_iros_flo/src/tom_robot/tom_configs/Arms/parameters/Scripts/progParkTOM_RA");

