#include "Planner.hpp"
#include <visualization_msgs/Marker.h>

#define DEBUG false
visualization_msgs::Marker points, line_strip, line_list;
geometry_msgs::Point p;
geometry_msgs::Point g;
Planner::Planner(ros::NodeHandle node1,Map_manager manager, int step_size)
{
  Cfree = manager.getCfree();

  branch_length = step_size;
  region_radius = step_size*3/2;

  map = manager;
}

// Return a random point from the Cspace
std::vector<float> Planner::get_random_point()
{
  //int random = randNum(0, Cfree.size());
  std::vector<float> floatVec;
  std::random_device rd;
  std::mt19937 gen(rd());
  
  std::uniform_real_distribution<> x(0, 384);
  std::uniform_real_distribution<> y(0, 384);
  floatVec.push_back(x(gen));
  floatVec.push_back(y(gen));

  return floatVec;
}

bool Planner::hasObstacle(std::vector<float> Xnear, std::vector<float> Xnew)
{
  bool result = false;

  float diff1 = (Xnew[0] - Xnear[0]);
  float diff2 = (Xnew[1] - Xnear[1]);

  int decimated_index;
  float diff;

  // take the greater difference
  if (fabs(diff1) > fabs(diff2))
  {
    diff = diff1;
    decimated_index = 1;
  }
  else
  {
    diff = diff2;
    decimated_index = 0;
  }

  // Creates set of points between two points
  std::vector<std::vector<float> > points_to_check;
  points_to_check.push_back(Xnear);

  for (int ii = 1; ii <= fabs(diff); ii++)
  {
    std::vector<float> point;
    point.push_back(Xnear[0] + ii * diff1 / fabs(diff));
    point.push_back(Xnear[1] + ii * diff2 / fabs(diff));

    point[decimated_index] = floor(point[decimated_index]);
    points_to_check.push_back(point);

    if (floor(point[decimated_index]) != point[decimated_index])
    {
      point[decimated_index]++;
      points_to_check.push_back(point);
    }
  }

  // returns true if one of the point in between is an obstacle
  for (int jj = 0; jj < points_to_check.size(); jj++)
  {
    std::vector<int> floatVec;
    floatVec.push_back(floor(points_to_check[jj][0])); 
    floatVec.push_back(floor(points_to_check[jj][1])); 
    if (map.checkObstacle(floatVec))
    {
      result = true;
    }
  }

  return result;
}

// returns the nearest node in the tree
std::vector<float> Planner::find_nearest(std::vector<float> Xrand)
{
  std::vector<float> Xnear;
  long min_distance = 1000;
  long distance;

  for (int ii = 0; ii < tree.size(); ii++)
  {
    distance = calculateDistance(Xrand, tree[ii].node);
    if (distance < min_distance)
    {
      min_distance = distance;
      Xnear = tree[ii].node;
    }
  }
  return Xnear;
}

// returns the new node displaced a particular branch length in the random node's direction
std::vector<float> Planner::new_node(std::vector<float> Xnear, std::vector<float> Xrand)
{
  std::vector<float> Xnew;
  float slope = (Xrand[1] - Xnear[1]) / (Xrand[0] - Xnear[0]);
  float adjuster = branch_length * sqrt(1 / (1 + pow(slope, 2)));

  std::vector<float> point1, point2;

  point1.push_back(Xnear[0] + adjuster);
  point1.push_back(Xnear[1] + slope * adjuster);

  point2.push_back(Xnear[0] - adjuster);
  point2.push_back(Xnear[1] - slope * adjuster);

  float distance1 = calculateDistance(Xrand, point1);
  float distance2 = calculateDistance(Xrand, point2);
  if (distance1 < distance2)
    Xnew = point1;
  else
    Xnew = point2;

  return Xnew;
}

// returns the index of nodes in neighbourhood
std::vector<int> Planner::get_neighbourhood(std::vector<float> Xnew)
{
  std::vector<int> neighbourhood;
  for (int i = 0; i < tree.size(); i++)
  {
    if (calculateDistance(tree[i].node, Xnew) < region_radius)
      neighbourhood.push_back(i);
  }
  return neighbourhood;
}

// returns the parent with least cost to come
std::vector<float> Planner::get_best_parent(std::vector<int> neighbourhood)
{
  float min = tree[neighbourhood[0]].costToCome;
  std::vector<float> Xnear = tree[neighbourhood[0]].node;
  int position = neighbourhood[0];
  for (int i = 1; i < neighbourhood.size(); i++)
  {
    if (min > tree[neighbourhood[i]].costToCome)
    {
      min = tree[neighbourhood[i]].costToCome;
      Xnear = tree[neighbourhood[i]].node;
      position = neighbourhood[i];
    }
  }
  Xnear.push_back(position);
  // The third index is the position in the tree
  return Xnear;
}

// returns the position of the parent
long Planner::findParent(long position_of_child)
{
  for (long i = 0; i < tree.size(); i++)
  {
    for (int j = 0; j < tree[i].branches.size(); j++)
    {
      if (tree[i].branches[j] == position_of_child)
        return i;
    }
  }
}

// returns random number between min and max
int Planner::randNum(int min, int max)
{
  return rand() % max + min;
}

// returns the euclidian distance between two points
float Planner::calculateDistance(std::vector<float> first_point, std::vector<float> second_point)
{
  return (float)sqrt((double)pow(first_point[0] - second_point[0], 2) + (double)pow(first_point[1] - second_point[1], 2));
}

std::vector<geometry_msgs::PoseStamped> Planner::RRTstarPlan(ros::NodeHandle node,std::vector<float> root, std::vector<float> target)
{
  std::vector<geometry_msgs::PoseStamped> plan;
	 ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  visualization_msgs::Marker points, line_strip, line_list;
     points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
     points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
     points.ns = line_strip.ns = line_list.ns = "Tree";
     points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
     points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
       points.id = 0;
     line_strip.id = 1;
     line_list.id = 2;

 
     points.type = visualization_msgs::Marker::POINTS;
     line_strip.type = visualization_msgs::Marker::LINE_STRIP;
     line_list.type = visualization_msgs::Marker::LINE_LIST;
 
 
 
     // POINTS markers use x and y scale for width/height respectively
     points.scale.x = 0.04;
     points.scale.y = 0.04;
 
     // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
     line_strip.scale.x = 0.0093;
     line_list.scale.x = 0.0093;
 
 
 
     // Points are green
     points.color.g = 1.0f;
     points.color.a = 1.0;
 
     // Line strip is blue
     line_strip.color.b = 1.0;
     line_strip.color.a = 1.0;
 
     // Line list is red
     line_list.color.r = 1.0;
     line_list.color.a = 1.0;
  Tree twig;
  twig.node = root;
  twig.costToCome = 0;
  tree.push_back(twig);

  std::vector<int> floaterVec;
  floaterVec.push_back(floor(target[0])); 
  floaterVec.push_back(floor(target[1])); 

  if(map.checkObstacle(floaterVec))
  {
    std::cout<<"Target in obstacle"<< std::endl;
    return plan;
  }

  distance_to_target = branch_length + 20;

  int count = 0;

  std::vector<int> neighbourhood;
  std::vector<float> Xnew, Xnear, Xnearest, Xrand, parent;
  int position;

  Xnew = root;
  std::cout << "------------- Starting Search ------------- " << std::endl;
  while ((distance_to_target > branch_length) || hasObstacle(target, Xnew) || count<10000)
  {
    count++;

    if(DEBUG) std::cout << "-- Current Count :" << count << "    -- distance to target :" << distance_to_target<< "    -- Has Obstacle :"<< hasObstacle(target, Xnew) <<std::endl;

    Xrand = get_random_point();
    
    Xnearest = find_nearest(Xrand);
    if(Xnearest[0]==Xrand[0] || Xnearest[1]==Xrand[1])
      continue;
    Xnew = new_node(Xnearest, Xrand);
    neighbourhood = get_neighbourhood(Xnew);
    parent = get_best_parent(neighbourhood);
    Xnear.clear();
    Xnear.push_back(parent[0]);
    Xnear.push_back(parent[1]);
    position = parent[2];

    // Add node if obstacle not in between
    std::vector<float> point1, point2;

    point1.push_back(floor(Xnear[0]));
    point1.push_back(floor(Xnear[1]));
    point2.push_back(floor(Xnew[0]));
    point2.push_back(floor(Xnew[1]));

    if (!hasObstacle(point1, point2))
    {
      long current_no_of_nodes = tree.size();
      
      if(DEBUG)
      {
        std::cout << "Printing nodes...\n";
        for (int cc = 0; cc < tree.size(); cc++)
        { 
          std::cout << "["<<tree[cc].node[0]<< ", "<<tree[cc].node[1]<< "] "<< std::endl;
        }
        std::cout << "Press any key to continue...\n";
        getchar();
      }

      // Add new node
      Tree temp;
      
      temp.node = Xnew;
      temp.costToCome = tree[position].costToCome + calculateDistance(Xnear, Xnew);

      tree.push_back(temp);
p.x = (Xnew[0]*0.05)-10 ;
       p.y = (Xnew[1]*0.05)-10;
      p.z = 0.0;
       //std::cout << "["<<p.x<< ", "<<p.y<< "] "<< std::endl;
       points.points.push_back(p);
       line_strip.points.push_back(p);
       line_list.points.push_back(p);
        p.x = (Xnear[0]*0.05)-10 ;
       p.y = (Xnear[1]*0.05)-10;
      p.z = 0.0;
      points.points.push_back(p);
       line_strip.points.push_back(p);
       line_list.points.push_back(p);
     marker_pub.publish(points);
     marker_pub.publish(line_list);
      // Add child location in parent node
      tree[position].branches.push_back(current_no_of_nodes);

      // check if close to target
      distance_to_target = calculateDistance(Xnew, target);

      // Rewiring
      for (int cc = 0; cc < neighbourhood.size(); cc++)
      {
        if ((tree[current_no_of_nodes].costToCome + calculateDistance(tree[neighbourhood[cc]].node, Xnew)) < tree[neighbourhood[cc]].costToCome)
        {
          // If cost from new node is cheaper switch parents
          long location = findParent(neighbourhood[cc]);
          tree[location].branches.erase(std::remove(tree[location].branches.begin(), tree[location].branches.end(), neighbourhood[cc]), tree[location].branches.end());
          tree[current_no_of_nodes].branches.push_back(neighbourhood[cc]);
        }
      }
    }
    
  }// end of search loop

  // Adding the target Node to the tree
  long current_no_of_nodes = tree.size();
  // Add new node
	points.points.clear();
	line_strip.points.clear();
	line_list.points.clear();
	marker_pub.publish(points);
	marker_pub.publish(line_strip);
	marker_pub.publish(line_list);
  Tree temp;

  temp.node = target;
  temp.costToCome = tree[current_no_of_nodes-1].costToCome + calculateDistance(target, Xnew);

  tree.push_back(temp);
	

  // Add child location in parent node
  tree[current_no_of_nodes-1].branches.push_back(current_no_of_nodes);

  //std::cout << "------------- Search Optimal Path ------------- " << std::endl;
  // Track the optimal path
  long node_number = current_no_of_nodes;
  std::vector<float> current_node = target;
  
  while (root[0] != current_node[0] || root[1] != current_node[1])
  {
	
    geometry_msgs::PoseStamped pos;
    pos.pose.position.x = ((current_node[0]*0.05) - 10);
    pos.pose.position.y = ((current_node[1]*0.05) - 10);
	p.x = pos.pose.position.x;
	p.y = pos.pose.position.y;
	p.z = 0.0;
       points.points.push_back(p);
       line_strip.points.push_back(p);
     marker_pub.publish(points);
     marker_pub.publish(line_strip);

    plan.push_back(pos);

    node_number = findParent(node_number);
    current_node = tree[node_number].node;
  }

  return plan;
}

std::vector<geometry_msgs::PoseStamped> Planner::RRTPlan(ros::NodeHandle node,std::vector<float> root, std::vector<float> target)
{
  	std::vector<geometry_msgs::PoseStamped> plan;
	 ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  	visualization_msgs::Marker points, line_strip, line_list;
     points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
     points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
     points.ns = line_strip.ns = line_list.ns = "Tree";
     points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
     points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
     points.id = 0;
     line_strip.id = 1;
     line_list.id = 2;

 
     points.type = visualization_msgs::Marker::POINTS;
     line_strip.type = visualization_msgs::Marker::LINE_STRIP;
     line_list.type = visualization_msgs::Marker::LINE_LIST;
 
 
 
     // POINTS markers use x and y scale for width/height respectively
     points.scale.x = 0.04;
     points.scale.y = 0.04;
 
     // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
     line_strip.scale.x = 0.0093;
     line_list.scale.x = 0.0093;
 
 
 
     // Points are green
     points.color.g = 1.0f;
     points.color.a = 1.0;
 
     // Line strip is blue
     line_strip.color.b = 1.0;
     line_strip.color.a = 1.0;
 
     // Line list is red
     line_list.color.r = 1.0;
     line_list.color.a = 1.0;

  	Tree twig;
	twig.node = root;
  	twig.costToCome = 0;
  	tree.push_back(twig);

  	std::vector<int> floaterVec;
  	floaterVec.push_back(floor(target[0])); 
  	floaterVec.push_back(floor(target[1])); 

  	if(map.checkObstacle(floaterVec))
  	{
    	std::cout<<"Target in obstacle"<< std::endl;
    	return plan;
  	}

  	std::vector<int> neighbourhood;
	std::vector<float> Xnew, Xnear,Xrand, parent;
  	int position;	

  	Xnew = root;
 	distance_to_target = branch_length + 20;

 	int count = 0;	
  	std::cout << "------------- Starting Search ------------- " << std::endl;
  	while ((distance_to_target > branch_length ) || hasObstacle(target, Xnew) || count<10000)
  	{
    	count++;
    	if(DEBUG) std::cout << "-- Current Count :" << count << "    -- distance to target :" << distance_to_target<< "    -- Has 		Obstacle :"<< hasObstacle(target, Xnew) <<std::endl;	

        // get a random point on the map
	Xrand = get_random_point();
      	//std::cout << "Random Points=["<<Xrand[0]<< ", "<<Xrand[1]<< "]"<< std::endl;
      	// find the closest known vertex to that point
    	Xnear = find_nearest(Xrand);
    	//std::cout << "Near Points=["<<Xnear[0]<< ", "<<Xnear[1]<< "]"<< std::endl;
    	if(Xnear[0]==Xrand[0] || Xnear[1]==Xrand[1])
      		continue;
        // try to move from the closest known vertex towards the random point
	Xnew = new_node(Xnear, Xrand);

    	//std::cout << "New Points=["<<Xnew[0]<< ", "<<Xnew[1]<< "]"<< std::endl;
	neighbourhood = get_neighbourhood(Xnew);
	parent = get_best_parent(neighbourhood);
	position = parent[2];
    	// Add node if obstacle not in between	
    	std::vector<float> point1, point2;	

    	point1.push_back(floor(Xnear[0]));
    	point1.push_back(floor(Xnear[1]));
    	point2.push_back(floor(Xnew[0]));
    	point2.push_back(floor(Xnew[1]));

		if (!hasObstacle(point1, point2))
	 	{
		  long current_no_of_nodes = tree.size();
		  //std::cout << "current="<<current_no_of_nodes<< std::endl;
        	    if(DEBUG)
        	    {
        		std::cout << "Printing nodes...\n";
        		for (int cc = 0; cc < tree.size(); cc++)
        		{ 
        	  	std::cout << "["<<tree[cc].node[0]<< ", "<<tree[cc].node[1]<< "] "<< std::endl;
        		}
        		std::cout << "Press any key to continue...\n";
        		getchar();
      		    }
	      	// Add new node
      		Tree temp;
      		temp.node = Xnew;
      		temp.costToCome = tree[position].costToCome + calculateDistance(Xnear, Xnew);
      		tree.push_back(temp);
	p.x = (Xnew[0]*0.05)-10 ;
       p.y = (Xnew[1]*0.05)-10;
      p.z = 0.0;
       //std::cout << "["<<p.x<< ", "<<p.y<< "] "<< std::endl;
       points.points.push_back(p);
       line_strip.points.push_back(p);
       line_list.points.push_back(p);
        p.x = (Xnear[0]*0.05)-10 ;
       p.y = (Xnear[1]*0.05)-10;
      p.z = 0.0;
      points.points.push_back(p);
       line_strip.points.push_back(p);
       line_list.points.push_back(p);
     marker_pub.publish(points);
     marker_pub.publish(line_list);
	
      		// Add child location in parent node
	        tree[position].branches.push_back(current_no_of_nodes);
	      	// check if close to target
      		distance_to_target = calculateDistance(Xnew, target);
	       }
     }	// end of search loop
  	// Adding the target Node to the tree
	points.points.clear();
	line_strip.points.clear();
	line_list.points.clear();
	marker_pub.publish(points);
	marker_pub.publish(line_strip);
	marker_pub.publish(line_list);

  	Tree temp;
	long current_no_of_nodes = tree.size();
	temp.node = target;
  	temp.costToCome = tree[current_no_of_nodes-1].costToCome + calculateDistance(target, Xnew);
	tree.push_back(temp);

	tree[current_no_of_nodes-1].branches.push_back(current_no_of_nodes);

 	long node_number = current_no_of_nodes;	
	std::vector<float> current_node = target;

  	while (root[0] != current_node[0] || root[1] != current_node[1])
  	{
    	geometry_msgs::PoseStamped pos;
    	pos.pose.position.x = ((current_node[0]*0.05) - 10);
    	pos.pose.position.y = ((current_node[1]*0.05) - 10);
	std::cout<<"x y"<<pos.pose.position.x<<" "<<pos.pose.position.y<<std::endl;
	p.x = pos.pose.position.x;
	p.y = pos.pose.position.y;
	p.z = 0.0;
       points.points.push_back(p);
       line_strip.points.push_back(p);
     marker_pub.publish(points);
     marker_pub.publish(line_strip);
    	plan.push_back(pos);

    	node_number = findParent(node_number);
    	current_node = tree[node_number].node;
  	}	

  return plan;
}

