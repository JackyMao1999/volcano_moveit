/************************************************* 
Copyright:Volcano Robot 
Author: 锡城筱凯
Date:2021-02-04 
Blog：https://blog.csdn.net/xiaokai1999
Description:
**************************************************/  
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/String.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/Float64Stamped.h>

using namespace std;

#define TIME_STEP 32    //时钟
#define NMOTORS 6       //机械臂电机数量
ros::NodeHandle *n;

static int controllerCount;
static std::vector<std::string> controllerList; 

ros::ServiceClient timeStepClient;          //时钟通讯客户端
webots_ros::set_int timeStepSrv;            //时钟服务数据

ros::ServiceClient set_velocity_client;
webots_ros::set_float set_velocity_srv;

ros::ServiceClient set_position_client;   
webots_ros::set_float set_position_srv; 

vector<double> armpositions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
vector<double> armvelocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static const char *armNames[NMOTORS] = {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
void on_goal(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as);

/*******************************************************
* Function name ：quit
* Description   ：退出函数
* Parameter     ：
        @sig   信号
* Return        ：无
**********************************************************/
void quit(int sig) {
    ROS_INFO("User stopped the '/volcano' node.");
    timeStepSrv.request.value = 0; 
    timeStepClient.call(timeStepSrv); 
    ros::shutdown();
    exit(0);
}

/*******************************************************
* Function name ：controllerNameCallback
* Description   ：控制器名回调函数，获取当前ROS存在的机器人控制器
* Parameter     ：
        @name   控制器名
* Return        ：无
**********************************************************/
// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) { 
    controllerCount++; 
    controllerList.push_back(name->data);//将控制器名加入到列表中
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());

}

int main(int argc, char **argv) 
{
    std::string controllerName;
    // create a node named 'ur5' on ROS network
    ros::init(argc, argv, "ur5e_action_server", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;

    signal(SIGINT, quit);

    // subscribe to the topic model_name to get the list of availables controllers
    ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
        ros::spinOnce();
    }
    ros::spinOnce();

    timeStepClient = n->serviceClient<webots_ros::set_int>("ur5e/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;

    // if there is more than one controller available, it let the user choose
    if (controllerCount == 1)
        controllerName = controllerList[0];
    else {
        int wantedController = 0;
        std::cout << "Choose the # of the controller you want to use:\n";
        std::cin >> wantedController;
        if (1 <= wantedController && wantedController <= controllerCount)
        controllerName = controllerList[wantedController - 1];
        else {
        ROS_ERROR("Invalid number for controller choice.");
        return 1;
        }
    }
    ROS_INFO("Using controller: '%s'", controllerName.c_str());
    // leave topic once it is not necessary anymore
    nameSub.shutdown();

    //初始化电机
    for (int i = 0; i < NMOTORS; ++i) {
        // velocity初始速度设置为0   
        set_velocity_client = n->serviceClient<webots_ros::set_float>(string("/ur5e/") + string(armNames[i]) + string("/set_velocity"));   
        set_velocity_srv.request.value = 0.21;   
        if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)     
            ROS_INFO("Max Velocity set to 0.21 for motor %s.", armNames[i]);   
        else     
            ROS_ERROR("Failed to call service set_velocity on motor %s.", armNames[i]);
    }

    Server server(*n, "follow_joint_trajectory", boost::bind(&on_goal,_1,&server), false);
    ROS_INFO("TrajectoryActionServer: Starting");
    server.start();
    // main loop
    while (ros::ok()) {
        // ROS_INFO(" successful.");  
        if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
        ROS_ERROR("Failed to call service time_step for next step.");
        break;
        }
         ros::spinOnce();
    }
    ros::shutdown();
    return 0;
}



void on_goal(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as){
    int PointsSize = goal->trajectory.points.size();
    ROS_INFO("data size %d",PointsSize);
    for (int i = 0; i < PointsSize; i++)
    {
        armpositions = goal->trajectory.points[i].positions;
        armvelocities = goal->trajectory.points[i].velocities;
        for (int j = 0; j < NMOTORS; j++)
        {
            set_position_client = n->serviceClient<webots_ros::set_float>(string("/ur5e/") + string(armNames[j]) + string("/set_position"));   
            set_position_srv.request.value = armpositions[j];
            if (set_position_client.call(set_position_srv) && set_position_srv.response.success)     
                ROS_INFO("Position set to %f for motor %s.", armpositions[j], armNames[j]);   
            else     
                ROS_ERROR("Failed to call service set_position on motor %s.", armNames[j]);
        }
    }
    
    as->setSucceeded();
}

