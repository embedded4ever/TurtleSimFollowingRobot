#include "../include/hw3/json.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <fstream>
#include <math.h>
#include <ros/ros.h>

namespace turtleSim {

    class Turtle {
    private:

        float dist;

        double err_sum;
        double linVel;
        double angVel;

        bool flag;

        const std::string& fName = "/home/volkan/catkin_ws/src/hw3/src/resources/config.json";

        const float Kp = 0.5f;
        const float Ki = 0.01f;
        const float Kh = 0.7f;

        std::ifstream f;

        ros::NodeHandle node;
        ros::Subscriber sub;
        ros::Subscriber subTurtle2;
        ros::Publisher pub;
        ros::Rate r;

        geometry_msgs::Twist msg;

        turtlesim::Pose myPose;
        turtlesim::Pose goalPose;

        void runPID(double& vErr, double& thetaErr)
        {
            auto p = vErr * this -> Kp;
            auto i = vErr * this -> Ki;
            this -> linVel = p + i;
            this -> err_sum += vErr;
            this -> angVel = this -> Kh * thetaErr;
        }

        void updatePose(const turtlesim::Pose& msg)
        {
            myPose = msg;
        }

        void turtle2Pose(const turtlesim::Pose& msg)
        {   
            goalPose = msg;
            this -> flag = true;
        }

        double getVelErr() 
        {
            double err = 0.0;

            auto d1 = this -> goalPose.x - this -> myPose.x;
            auto d2 = this -> goalPose.y - this -> myPose.y;
            err = std::sqrt(std::pow(d1, 2) + std::pow(d2, 2)) - this -> dist;

            return err;
        }

        double getThetaErr() 
        {
            double err = 0.0;

            auto d1 = this -> goalPose.x - this -> myPose.x;
            auto d2 = this -> goalPose.y - this -> myPose.y;
            auto thetaGoal = std::atan2(d2, d1);
            auto temp = thetaGoal - this -> myPose.theta;
            err = atan2(std::sin(temp), std::cos(temp));

            return err;        
        }    
            
    public:
        Turtle() : f(fName), r(5), err_sum(0), linVel(0), angVel(0), flag(false) {
            if (f) {
                auto jfile = json::parse(f);
                auto jval = jfile["FollowerRobot"]["Dist"];
                this -> dist = json::to_number<float>(jval);
                ROS_INFO_STREAM("Config Distance : " << this -> dist);                               
            }
            else {
                ROS_ERROR_STREAM("Json file not found");
                throw 1;      
            }

            this -> sub = node.subscribe("/turtle1/pose", 1, &Turtle::updatePose, this);
            this -> subTurtle2 = node.subscribe("/turtle2/pose", 1, &Turtle::turtle2Pose, this);
            this -> pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        }

        void exec() 
        {
            double velErr = 0;
            double thetaErr = 0;

            while(ros::ok()) {

                ros::spinOnce();

                if (flag) 
                {                
                    velErr = this -> getVelErr();
                    thetaErr = this -> getThetaErr();
                    this -> runPID(velErr, thetaErr);

                    msg.linear.x = this -> linVel;
                    msg.angular.z = this -> angVel;

                    pub.publish(msg);
                    
                    r.sleep();
                    flag = false;
                }            
            }
        }
    };
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "turtle1");
    try {
        turtleSim::Turtle t;
        t.exec();   
    }catch(int i){
        ROS_ERROR_STREAM("Json file not found");
    }
       
}