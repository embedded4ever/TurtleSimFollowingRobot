#include "../include/hw3/json.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <fstream>
#include <ros/ros.h>
#include <turtlesim/Spawn.h> 
#include <math.h>
#include <random>

#define RANDOM_MIN 2
#define RANDOM_MAX 8

namespace turtleSim {

   class Turtle {
   private:

      float linVel;
      float angVel;

      const std::string& fName = "/home/volkan/catkin_ws/src/hw3/src/resources/config.json";
      
      //Json file
      std::ifstream f; 

      ros::NodeHandle node;
      ros::Publisher pub;
      ros::Subscriber sub;
      ros::ServiceClient client;

      turtlesim::Spawn srv;
      turtlesim::Spawn sc;
      turtlesim::Pose myPose;      

      geometry_msgs::Twist msg;

      //Update our pose variable when hit the callback
      void poseCallBack(const turtlesim::Pose& msg)
      {
         myPose = msg;
      } 

      constexpr double pi() const
      { 
         return std::atan(1)*4; 
      }
            
   public:
      Turtle() : f(fName), linVel(0), angVel(0) {
         if (f) {
            auto jfile = json::parse(f);
            auto jval = jfile["ControlRobot"]["LinearVel"];
            this -> linVel = json::to_number<float>(jval);
            ROS_INFO_STREAM("Linear Velocity : " << this -> linVel);  

            jval = jfile["ControlRobot"]["AngularVel"];
            this -> angVel = json::to_number<float>(jval);
            ROS_INFO_STREAM("Angular Velocity : " << this -> angVel);                                
         }
         else {
            ROS_ERROR_STREAM("Json file not found");
            throw 1;        
         }    
         
         client = node.serviceClient<turtlesim::Spawn>("spawn");
         
         std::mt19937 gen{ std::random_device{}() };
         std::uniform_real_distribution<float> dist(RANDOM_MIN, RANDOM_MAX);

         srv.request.x = dist(gen);
         srv.request.y = dist(gen);
         srv.request.theta = 1.5;
         srv.request.name = "turtle2";
         //Get new turtle
         client.call(srv);

         this -> pub = node.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 1);
         this -> sub = node.subscribe("/turtle2/pose", 1, &Turtle::poseCallBack, this);
      }

      ~Turtle() = default;

      //Robot hit the wall ?      
      void hitWall(double angVel)
      {
         double t = 0;

         if (this -> myPose.x < 1 && this -> myPose.theta > 0)
         {
            t = pi() - this -> myPose.theta;
            changeDir(t);
         }

         else if (this -> myPose.x < 1 && this -> myPose.theta < 0)
         {
            t = (-1 * pi()) - this -> myPose.theta;
            changeDir(t); 
         }

         else if (this -> myPose.x > 11 && this -> myPose.theta > 0)
         {
            t = pi() - this -> myPose.theta;
            changeDir(t);    
         }

         else if (this -> myPose.x > 11 && this -> myPose.theta < 0)
         {
            t = (-1 * pi()) - this -> myPose.theta;
            changeDir(t);
         }

         else if (this -> myPose.y > 11)
         {
            t = (-1 * this -> myPose.theta);
            changeDir(t);
         }

         else if (this -> myPose.y < 0.1)
         {
            t = (-1 * this -> myPose.theta);
            changeDir(t);  
         }     
      } 

      void changeDir(double t)
      {
         auto d = std::abs(this -> myPose.theta - t);

         msg.angular.z = (d > 0.1 ) ? this -> angVel : 0;

         pub.publish(msg);
      }   

      void exec() 
      {
         ros::Rate r(5);

         while(ros::ok()) {         
            
            this -> msg.linear.x = this -> linVel;
            pub.publish(msg);

            this -> hitWall(this -> angVel);
            ros::spinOnce();   
            r.sleep();
         }
      }
   };
}

int main(int argc, char** argv)
{   
   ros::init(argc, argv, "turtle2");
   try {
        turtleSim::Turtle t;
        t.exec();   
   }catch(int i){
        ROS_ERROR_STREAM("Json file not found");
   }  
}