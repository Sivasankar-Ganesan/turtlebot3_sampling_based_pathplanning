#include "Map_manager.hpp"
#include "Planner.hpp"
#include "matplotlibcpp.h"
#include "controller.hpp"

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <geometry_msgs/Twist.h>

#include <cmath>

#define DEBUG false
#define STEP_SIZE 3

namespace plt = matplotlibcpp;
using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot3_sampling_based_pathplanning_node");

  ros::NodeHandle node;

  // Initialization
  //cout << "Inside Map Manager" << endl;
  Map_manager manager;
  //cout << "Inside Planner" << endl;
  Planner planner(node,manager, STEP_SIZE);
  //cout << "Outside Planner" << endl;
  controller control(node, STEP_SIZE);
  //cout << "Outside Controller" << endl;

  // Initialized position corresponds to the spawn position in gazebo environment
  vector<float> start_state, end_state;
  vector<float> start_point, target_point;

  start_state.push_back(-2.0);
  start_state.push_back(0);

  end_state.push_back(2.0);
  end_state.push_back(0);
  end_state.push_back(0);

  float final_yaw = end_state[2];

  target_point = manager.computeGridCoordinate(end_state);
  start_point = manager.computeGridCoordinate(start_state);

   cout << "Start Point: " << start_point[0] << ", " << start_point[1] << endl;
   cout << "End Point: " << target_point[0] << ", " << target_point[1] << endl;

  if (DEBUG)
  {
    cout << "Start Point: " << start_point[0] << ", " << start_point[1] << endl;
    cout << "End Point: " << target_point[0] << ", " << target_point[1] << endl;
  }

  // Make the plan with RRT
  	std::vector<geometry_msgs::PoseStamped> plan;
 	char mode = '0';
 	std::cout << "Please choose method of path planning" << std::endl;
        std::cout << "1. RRT" << std::endl;
        std::cout << "2. RRT*" << std::endl;
        std::cin >> mode;
	switch(mode) 
	{
        case '1': 
	plan = planner.RRTPlan(node,start_point, target_point);
	break;
        case '2': 
	plan = planner.RRTstarPlan(node,start_point, target_point);
	break;
	}  

  int counter = 0;

  ROS_INFO_STREAM("Plotting map with Global Plan");
  // Plot the map
 /* for(int i=0;i<384;i++)
    for(int j=0;j<384;j++)
      if(manager.get_state(i,j)>150)
      {
        vector<double> x,y;
        x.push_back((i*0.05) - 10);
        x.push_back((i*0.05) - 10+0.01);
        y.push_back((j*0.05) - 10);
        y.push_back((j*0.05) - 10+0.01);
        plt::plot(x, y);
        if (DEBUG) cout<<"Printing"<<counter++<<endl;
      }    
  

  // Plot the plan
  vector<double> x,y;
  for(int i=plan.size()-1; i >= 0; i--)
  {
    x.push_back(plan[i].pose.position.x);
    y.push_back(plan[i].pose.position.y);
  }
  plt::plot(x, y);
  plt::xlim(-10, 10);
  plt::ylim(-10, 10);
  plt::show();*/
  
  control.executePlan(plan, final_yaw);

  return 0;
}

