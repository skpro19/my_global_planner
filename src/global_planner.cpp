#include <pluginlib/class_list_macros.h>
#include <my_global_planner/global_planner.h>
#include <tf/tf.h>
#include <queue>
#include<math.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace global_planner {

  GlobalPlanner::GlobalPlanner (){

  }

  GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);

  }


  void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

    ros::NodeHandle nh_ = ros::NodeHandle{"~abcd"};

    costmap_ros  = costmap_ros;
    costmap_ros_ = costmap_ros->getCostmap(); 

    size_x = costmap_ros_->getSizeInCellsX(); 
    size_y = costmap_ros_->getSizeInCellsY();

    cout << "global_frame: " << costmap_ros->getGlobalFrameID() << endl;

    global_plan_pub = nh_.advertise<nav_msgs::Path>("my_global_path", 1 );

  }


  double GlobalPlanner::heu(Point p1, Point p2) {

    if(p1.x < p2.x) {swap(p1.x, p2.x);}
    if(p1.y < p2.y) {swap(p1.y, p2.y);}

    __uint32_t sum = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);

      return (sum);

  }


  bool GlobalPlanner::makePlanThree(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    cout << "Inside the make_plan function!" << endl;

    //bool reached  = false;
    
    __uint32_t mx_i, my_i, mx_f, my_f;

    double wx_i, wy_i, wx_f, wy_f;

    wx_i = start.pose.position.x; 
    wy_i = start.pose.position.y;
    wx_f = goal.pose.position.x;
    wy_f = goal.pose.position.y;

    costmap_ros_->worldToMap(wx_i, wy_i, mx_i, my_i);
    costmap_ros_->worldToMap(wx_f, wy_f, mx_f, my_f);

    cout << endl;
    cout << "Printing start pose and goal pose: " << endl;
    cout << "w_i: (" << wx_i << "," << wy_i << ") m_i: (" << mx_i << "," << my_i <<")" << endl;;
    cout << "w_f: (" << wx_f << "," << wy_f << ") m_f: (" << mx_f << "," << my_f <<")" << endl;;
    cout << endl;

    cout << "Sleeping for 1 second!" << endl;
    ros::Duration(1.0).sleep();
    cout << endl;

    //nav_msgs::Path

    return true;
    
  }

  bool GlobalPlanner::makePlanOne(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    plan.push_back(start);
    for (int i=0; i<20; i++){
    geometry_msgs::PoseStamped new_goal = goal;
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

    new_goal.pose.position.x = -2.5+(0.05*i);
    new_goal.pose.position.y = -3.5+(0.05*i);

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();

    plan.push_back(new_goal);
    }
    plan.push_back(goal);
    return true;
  
  }

  void GlobalPlanner::update_planner_plan(vector<Point> &path_points, vector<geometry_msgs::PoseStamped> &plan, const geometry_msgs::PoseStamped &goal) {

    for(int i =0 ; i < (int)path_points.size(); i++) {

      geometry_msgs::PoseStamped curr_pt = goal;

      double wx_c, wy_c; 
      __uint32_t mx_c = path_points[i].x, my_c = path_points[i].y;

      costmap_ros_->mapToWorld(mx_c, my_c, wx_c, wy_c);

      curr_pt.pose.position.x = wx_c;
      curr_pt.pose.position.y = wy_c;      

      plan.push_back(curr_pt);

    }
  }

  bool GlobalPlanner::makePlanTwo(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    cout << "Inside the make_plan function!" << endl;

    cout << "Hello World!" << endl;
    
    __uint32_t mx_i, my_i, mx_f, my_f;

    double wx_i, wy_i, wx_f, wy_f;

    wx_i = start.pose.position.x; 
    wy_i = start.pose.position.y;
    wx_f = goal.pose.position.x;
    wy_f = goal.pose.position.y;

    costmap_ros_->worldToMap(wx_i, wy_i, mx_i, my_i);
    costmap_ros_->worldToMap(wx_f, wy_f, mx_f, my_f);

    cout << endl;
    cout << "Printing start pose and goal pose: " << endl;
    cout << "w_i: (" << wx_i << "," << wy_i << ") m_i: (" << mx_i << "," << my_i <<")" << endl;;
    cout << "w_f: (" << wx_f << "," << wy_f << ") m_f: (" << mx_f << "," << my_f <<")" << endl;;
    cout << endl;

    vector<Point> path_points;
    path_points.clear();

    __uint32_t mx_last = mx_i, my_last =  my_i;






    //vis_points.insert(best_pt);

    for(int i =0 ; i < (int)path_points.size(); i++) {

      geometry_msgs::PoseStamped curr_pt = goal;

      double wx_c, wy_c; 
      __uint32_t mx_c = path_points[i].x, my_c = path_points[i].y;

      costmap_ros_->mapToWorld(mx_c, my_c, wx_c, wy_c);

      curr_pt.pose.position.x = wx_c;
      curr_pt.pose.position.y = wy_c;      

      plan.push_back(curr_pt);

    }

    nav_msgs::Path my_path = {};

    my_path.header.frame_id = goal.header.frame_id;
    my_path.header.stamp = ros::Time::now();

    my_path.poses = plan;

    global_plan_pub.publish(my_path);

    cout << "path_points.size(): " << path_points.size() << endl;
    cout << "plan.size(): " << plan.size() << endl;

    //cout << "Sleeping for 3 seconds!" << endl; 
    //ros::Duration(3.0).sleep();

    return true;

  }

  void GlobalPlanner::publish_global_path(const vector<geometry_msgs::PoseStamped> &plan, const geometry_msgs::PoseStamped &goal) {

    nav_msgs::Path my_path = {};

    my_path.header.frame_id = goal.header.frame_id;
    my_path.header.stamp = ros::Time::now();

    my_path.poses = plan;

    global_plan_pub.publish(my_path);

  }

  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    cout << "Inside the make_plan function!" << endl;

    cout << "Hello World!" << endl;
    
    __uint32_t mx_i, my_i, mx_f, my_f;

    double wx_i, wy_i, wx_f, wy_f;

    wx_i = start.pose.position.x; 
    wy_i = start.pose.position.y;
    wx_f = goal.pose.position.x;
    wy_f = goal.pose.position.y;

    costmap_ros_->worldToMap(wx_i, wy_i, mx_i, my_i);
    costmap_ros_->worldToMap(wx_f, wy_f, mx_f, my_f);

    cout << endl;
    cout << "Printing start pose and goal pose: " << endl;
    cout << "w_i: (" << wx_i << "," << wy_i << ") m_i: (" << mx_i << "," << my_i <<")" << endl;;
    cout << "w_f: (" << wx_f << "," << wy_f << ") m_f: (" << mx_f << "," << my_f <<")" << endl;;
    cout << endl;

    vector<Point> path_points;
    path_points.clear();

    __uint32_t mx_last = mx_i, my_last =  my_i;


    while(true){ 
      
   
      if(mx_last >= mx_f || my_last >= my_f) {

        cout << "Almost reached the goal -- -breaking from the while loop!" << endl; 
        break;

      }

      if(Point{mx_last, my_last} == Point{mx_f, my_f}) {
      
        cout << "Reached the goal! ------  breaking from the while loop" << endl; 
        break;
        
      }

      path_points.push_back(Point{mx_last, my_last});
      
      double mn_dis = UINT32_MAX;

      Point best_pt = {};

      //cout << "Going inside the for loop!" << endl;

      for(int i = (int)mx_last + 1; i >=(int)mx_last; i--) {

        for(int j = (int)my_last + 1; j>= (int)my_last ; j--) {
          
          
          if(i < 0 || j<0 || i>= size_x || j >= size_y ) {continue;}

          if(i == mx_last && j == my_last) {continue;}

          unsigned char cell_cost = costmap_ros_->getCost(i,j);

          if(cell_cost == costmap_2d::LETHAL_OBSTACLE || cell_cost == costmap_2d::NO_INFORMATION) {continue;}

      
          //if(vis_points.find(Point{(__uint32_t)i,(__uint32_t)j}) != vis_points.end()) {continue;}

          //vis_points.insert(Point{(__uint32_t)i, (__uint32_t)j});

          double dis_from_goal = heu(Point{i,j}, Point{goal.pose.position.x, goal.pose.position.y});

          cout << "(" << i <<"," << j <<"): " << dis_from_goal << endl;

          if(dis_from_goal < mn_dis) {

              mn_dis = dis_from_goal;
              best_pt = Point{(__uint32_t)i , (__uint32_t)j};

          }

        }

      }

      //cout << endl;
      //cout << "Sleeping for 5 seconds!" << endl;
      //ros::Duration(5.0).sleep();

      mx_last = best_pt.x , my_last = best_pt.y;

      //cout << "best_pt: (" << best_pt.x << "," << best_pt.y <<") mn_dis: " << mn_dis << endl << endl;

      //cout << "Sleeping for 3 seconds!" << endl; 
      //ros::Duration(3.0).sleep();



    }

    //vis_points.insert(best_pt);

    update_planner_plan(path_points, plan, goal);

    publish_global_path(plan, goal);
    
    
    return true;

  }
  


};
