#include <pluginlib/class_list_macros.h>
#include <my_global_planner/global_planner.h>
#include <tf/tf.h>
#include <queue>
#include<math.h>
#include <cstdlib>
#include <ctime>
#include <random>
#include<visualization_msgs/Marker.h>

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

    nh_ = ros::NodeHandle{"~abcd"};
    
    my_costmap_ros  = costmap_ros;
    costmap_ros_ = costmap_ros->getCostmap(); 

    size_x = costmap_ros_->getSizeInCellsX(); 
    size_y = costmap_ros_->getSizeInCellsY();

    
    global_plan_pub = nh_.advertise<nav_msgs::Path>("my_global_path", 1 );
    goal_marker_pub = nh_.advertise<visualization_msgs::Marker>("goal_markers", 10);

    marker_id_cnt = 0 ;

    update_map_bounds();


    //pose_sub = nh_.subscribe("/camera/depth_registered/points", 1000, &GlobalPlanner::initialpose_callback, this);
 
    //pose_sub = nh_.subscribe("/camera/depth_registered/points", 1000, boost::bind(&GlobalPlanner::initialpose_callback, this, _1));

  }


  void GlobalPlanner::update_map_bounds(){

    __uint32_t x_mn, x_mx, y_mn, y_mx;
    x_mn = size_x, y_mn = size_y;

    x_mx = 0 , y_mx = 0 ;

    for(__uint32_t i =0 ; i < size_x; i++) {

      for(__uint32_t j = 0 ; j < size_y; j++) {
        
        unsigned cell_cost = costmap_ros_->getCost(i, j);

        if(cell_cost != costmap_2d::NO_INFORMATION) {

          x_mn = min(x_mn, i);
          x_mx = max(x_mx, i); 
          y_mn = min(y_mn, j);
          y_mx = max(y_mx, j);

        }
        

      }

    }

    cout << "x_mx: " << x_mn << " x_mn: " << x_mx << endl;
    cout << "y_mn: " <<y_mn << " y_mx: " << y_mx << endl;
    
    map_xi = x_mn, map_xf = x_mx; 
    map_yi = y_mn, map_yf = y_mx;
  
    
  }

  double GlobalPlanner::heu(Point p1, Point p2) {

    cout << "heu function called!" << endl;
    if(p1.x < p2.x) {swap(p1.x, p2.x);}
    if(p1.y < p2.y) {swap(p1.y, p2.y);}

    __uint32_t sum = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);

      return (sqrt(sum));

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

  void GlobalPlanner::publish_global_path(const vector<geometry_msgs::PoseStamped> &plan, const geometry_msgs::PoseStamped &goal) {

    nav_msgs::Path my_path = {};

    my_path.header.frame_id = goal.header.frame_id;
    my_path.header.stamp = ros::Time::now();

    my_path.poses = plan;

    global_plan_pub.publish(my_path);

  }

  bool GlobalPlanner::print_cell(const Cell &cell) {

    cout << "cell.point: (" << cell.point.x <<"," << cell.point.y <<") cost: " << cell.cost_till_now << endl;

  }
  
  void GlobalPlanner::print_world_params(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, __uint32_t &mx_i, __uint32_t &my_i, __uint32_t &mx_f, __uint32_t &my_f){

    double wx_i, wy_i, wx_f, wy_f;

    wx_i = start.pose.position.x; 
    wy_i = start.pose.position.y;
    wx_f = goal.pose.position.x;
    wy_f = goal.pose.position.y;

    costmap_ros_->worldToMap(wx_i, wy_i, mx_i, my_i);
    costmap_ros_->worldToMap(wx_f, wy_f, mx_f, my_f);

    cout << "Printing start pose and goal pose: " << endl;
    cout << "w_i: (" << wx_i << "," << wy_i << ") m_i: (" << mx_i << "," << my_i <<")" << endl;;
    cout << "w_f: (" << wx_f << "," << wy_f << ") m_f: (" << mx_f << "," << my_f <<")" << endl;;
    cout << endl;

  }

  
  GlobalPlanner::Point GlobalPlanner::generate_next_goal(const Point &last_pose ){

      cout << "Inside the generate_next_goal function!" << endl;
      cout << "Sleeping for 2 seconds!" << endl; 

      //ros::Duration(2.0).sleep();

      std_msgs::ColorRGBA blue;
      blue.r = 0;
      blue.g = 0;
      blue.b = 1.0;
      blue.a = 1.0;
      std_msgs::ColorRGBA red;
      red.r = 1.0;
      red.g = 0;
      red.b = 0;
      red.a = 1.0;
      std_msgs::ColorRGBA green;
      green.r = 0;
      green.g = 1.0;
      green.b = 0;
      green.a = 1.0;

      Point nxt_point = {};

      bool point_found = true;



      while(true) {

        bool obs_found = 0 ;

        __uint32_t mx_ , my_, mx_i, my_i;
        double wx_, wy_;

        mx_i = last_pose.x , my_i = last_pose.y;

        cout << "last_pose.x: " << mx_i  << " last_pose.y: " << my_i;
        //cout << "Sleeping for 1 second!" << endl;
        //ros::Duration(1.0).sleep();

        //costmap_ros_->worldToMap(wx_i, wy_i, mx_i, my_i);
        //costmap_ros_->worldToMap(wx_goal, wy_goal, mx_goal, my_goal);        

        int step_size = 400;

        mx_= rand() % (((int)mx_i + step_size- (int)mx_i + step_size) + 1) + (mx_i -step_size);
        my_= rand() % (((int)my_i + step_size - (int)my_i + step_size) + 1) + (my_i -step_size);
       

        cout << endl;
        cout << "mx_i: " << mx_i << " my_i: " << my_i << endl;
        cout << "mx_: " << mx_ << " my_: " << my_ << endl;
        cout << endl;

        if(mx_ >= map_xf || mx_ < map_xi || my_ >= map_yf || my_ < map_yi) {

          cout << "out of bound point found! ---- SKIPPING" << endl;
          continue;

        }

        cout  << "mx_: " << mx_ << " my_: " << my_ << endl;

        costmap_ros_->mapToWorld(mx_, my_, wx_, wy_);

        cout << "Helllo!" << endl;
        cout << "wx_: " << wx_ << " wy_: " << wy_ << endl;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = nh_.getNamespace();

        marker.id = marker_id_cnt++;

        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = wx_;
        marker.pose.position.y = wy_;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.lifetime = ros::Duration(5.0);

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        
        marker.color.a = 1.0; // Don't forget to set the alpha!
        
        marker.color = green;

        unsigned char cell_cost = costmap_ros_->getCost(mx_, my_);
        cout << "cell_cost: " << cell_cost << endl;
        cout << "Hello2!" << endl;

        if(cell_cost == costmap_2d::LETHAL_OBSTACLE) { 

          obs_found = 1;  
          cout << "Point lies on an obstacle!" << endl;
          marker.color = red;
          marker.scale.x = 0.5; 
          marker.scale.y = 0.5;
          goal_marker_pub.publish(marker);
          continue;

        }

        cout << "obs_found: " << obs_found << endl;

        double dis = heu(Point{mx_, my_}, Point{mx_i, my_i});

                
        goal_marker_pub.publish( marker );        
        cout << "Hello3!" << endl;

        
        point_found = true;    
        nxt_point = Point{mx_, my_};
        cout << "Found next point: (:" << nxt_point.x <<"," << nxt_point.y <<")" << endl;
        
        //cout << "Sleep for 1 second!" << endl;
        //ros::Duration(1.0).sleep();
        break;
        
        

        cout << "Hello4" << endl;
      

      }

      if(!point_found) {

        cout << "No suitable point found ---- ERROR!" << endl;
        cout << "Sleeping for 5 seconds!" << endl;
        ros::Duration(5.0);      
      }

      return nxt_point;      
  
  }

  GlobalPlanner::RRT_Cell* GlobalPlanner::get_next_best_cell(const Point &curr_point, RRT_Cell* last_cell){


    cout << "Inside the get_next_best_cell function!" << endl;

      RRT_Cell* curr_cell = last_cell;

      
      double mn_dis = UINT32_MAX; 
      RRT_Cell* best_cell;

      int cnt = 0;

      while(true) {

        cnt++;
        cout << "cnt: " << cnt << endl;

        cout << "curr_cell == NULL: " << (curr_cell == NULL) << endl;
        
        if(curr_cell == NULL) {
          
          
          cout << "nullptr found!" << endl;
          cout << "best_cell->point.x: " << best_cell->point.x << " best_cell->point.y: "<< best_cell->point.y << endl;
          break;

        }

        Point pt = curr_cell->point;
        
        cout <<"H7" << endl;
        double dis = heu(pt, curr_point);
        cout << "dis: " << dis << endl;
        cout << "H6" << endl;

        if(dis < mn_dis) {

          mn_dis = dis;
          best_cell = curr_cell;

        }

        cout << "H4" << endl;
        curr_cell = curr_cell->parent;
        cout << "H5" << endl;

      }

      cout << "H0" << endl;
      return best_cell;


  }

  bool GlobalPlanner::RRT_path_so_far(RRT_Cell *last_cell) {

    RRT_Cell *curr_cell = last_cell;

    cout << "path_so_far: ";
    while(true){

      if(curr_cell == NULL) {break;}  
      Point pt = curr_cell->point;

      cout << "(" << pt.x << "," <<  pt.y <<", " << endl;

      curr_cell = curr_cell->parent;

    }


  }

  void GlobalPlanner::publish_marker_point(const Point &pt) {

    //cout << "Sleeping for 1 second!" << endl;
    //ros::Duration(1.0).sleep();
    std_msgs::ColorRGBA blue;
      blue.r = 0;
      blue.g = 0;
      blue.b = 1.0;
      blue.a = 1.0;
      std_msgs::ColorRGBA red;
      red.r = 1.0;
      red.g = 0;
      red.b = 0;
      red.a = 1.0;
      std_msgs::ColorRGBA green;
      green.r = 0;
      green.g = 1.0;
      green.b = 0;
      green.a = 1.0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();

    marker.id = marker_id_cnt++;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = pt.x;
    marker.pose.position.y = pt.y;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.5;
    
    marker.lifetime = ros::Duration(10.0);

    marker.color.a = 1.0; // Don't forget to set the alpha!
    
    marker.color = red;

    goal_marker_pub.publish(marker);

    //cout << "Sleeping for 1 second!" << endl;
    //ros::Duration(1.0).sleep();

  }

  void GlobalPlanner::update_RRT_path_points(vector<Point> &path_points, RRT_Cell* last_cell){

    
    RRT_Cell* curr_cell= last_cell;

    while(true){

      if(curr_cell == NULL) {return;}

      path_points.push_back(curr_cell->point);
      curr_cell = curr_cell->parent;

    }

  }

  /* 
  bool GlobalPlanner::makePlanOne(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

      int cnt = 0;
      bool reached = false;

    __uint32_t mx_i, my_i, mx_f, my_f, mx_c, my_c;

    print_world_params(start, goal, mx_i, my_i, mx_f, my_f);

    cout << "End of print_world_params function!" << endl;

    mx_c = mx_i, my_c = my_i;

    RRT_Cell* head_cell = new RRT_Cell();
    head_cell->point = Point{mx_i, my_i};
    head_cell->next_cell;
    head_cell->prev_cell=NULL;

    while(true){

      cout << "cnt: " << cnt << endl;
      cout << "Trying to generate_next_goal(): " << endl;

      Point curr_p = generate_next_goal(Point{mx_c, my_c});

      mx_c = curr_p.x , my_c = curr_p.y;

      double dis_from_goal = heu(curr_p, Point{mx_f, my_f});

      if(dis_from_goal < 30) {

        reached = true;
        cout << "Almost reached the goal!" << endl;
        cout << "Sleeping for 5 seconds!" << endl;
        ros::Duration(5.0).sleep();

        break;
      }  
      
      cout << "End of generate_next_goal function!" << endl << endl;

      RRT_Cell* best_cell = get_next_best_cell(curr_p, head_cell);
      
      cout << "best_cell->point.x: " << best_cell->point.x << " best_cell->point.y: " << best_cell->point.y << endl;

      RRT_Cell* new_next_cell = new RRT_Cell();
      //new_next_cell->next_cell = old_next_cell;
      new_next_cell->point = curr_p;
      new_next_cell->prev_cell = best_cell;

      if(best_cell -> next_cell == NULL) {

        best_cell->next_cell = new_next_cell;

      }
      else {
        
        new_next_cell->next_cell= best_cell->next_cell;
        best_cell->next_cell = new_next_cell;

      }

    }

    
    if(!reached) {

      cout << "Something is wrong! ---- Could not reach near the goal!" << endl;

    }

    RRT_path_so_far(head_cell);

    vector<Point> path_points;

    update_RRT_path_points(path_points, head_cell);

    update_planner_plan(path_points,plan, goal);  

    publish_global_path(plan, goal);

    cout << "Sleeping for 5 seconds!" << endl;
    ros::Duration(5.0).sleep();



    return true;

  }
  
  */


  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

      
      int cnt = 0;
      bool reached = false;

    __uint32_t mx_i, my_i, mx_f, my_f, mx_c, my_c;

    print_world_params(start, goal, mx_i, my_i, mx_f, my_f);

    cout << "End of print_world_params function!" << endl;

    mx_c = mx_i, my_c = my_i;

    

    RRT_Cell* head_cell = new RRT_Cell();
    head_cell->point = Point{mx_i, my_i};
    head_cell->parent =  NULL;

    RRT_Cell *last_cell = head_cell;
    
    while(true){

      cout << "Sleep for 1 second!" << endl;
      ros::Duration(1.0).sleep();

      cout << "cnt: " << cnt << endl;
      cout << "Trying to generate_next_goal(): " << endl;

      Point curr_p = generate_next_goal(Point{mx_c, my_c});

      mx_c = curr_p.x , my_c = curr_p.y;

      double dis_from_goal = heu(curr_p, Point{mx_f, my_f});

      if(dis_from_goal < 30) {

        reached = true;
        cout << "Almost reached the goal!" << endl;
        cout << "Sleeping for 5 seconds!" << endl;
        ros::Duration(5.0).sleep();

        break;
      }  
      
      cout << "End of generate_next_goal function!" << endl << endl;
      
      RRT_Cell* best_cell = get_next_best_cell(curr_p, last_cell);
      cout << "next_best_cell found! -- sleeping for 1 second! " << endl;
      ros::Duration(1.0).sleep();

      last_cell->point = curr_p;
      last_cell->parent = best_cell;



    }

    
    if(!reached) {

      cout << "Something is wrong! ---- Could not reach near the goal!" << endl;

    }

    RRT_path_so_far(last_cell);

    vector<Point> path_points;

    update_RRT_path_points(path_points, last_cell);

    update_planner_plan(path_points,plan, goal);  

    publish_global_path(plan, goal);

    cout << "Sleeping for 5 seconds!" << endl;
    ros::Duration(5.0).sleep();



    return true;

  }
  

};
