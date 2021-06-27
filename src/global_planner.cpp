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

    if(p1.x < p2.x) {swap(p1.x, p2.x);}
    if(p1.y < p2.y) {swap(p1.y, p2.y);}

    __uint32_t sum = ((int)p1.x - (int)p2.x) * ((int)p1.x - (int)p2.x) + ((int)p1.y - (int)p2.y) * ((int)p1.y - (int)p2.y);

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

  bool GlobalPlanner::check_cell_neighbour(const Point &pt, int margin_sz){

    //int margin_sz = 5;

    

    int is_reachable = 1;

    for(int i = (int)pt.x - margin_sz; i <= (int)pt.x + margin_sz; i++) {
      
      for(int j = (int)pt.y - margin_sz; j <= (int)pt.y + margin_sz; j++) {

        if(i <= map_xi || i >= map_xf || j <= map_yi || j >= map_yf) {is_reachable = 0;}

        unsigned char cell_cost = costmap_ros_->getCost(i, j);

        if(cell_cost == costmap_2d::LETHAL_OBSTACLE) {is_reachable = 0 ;}

      }

    }

    return is_reachable;

  }
  
  GlobalPlanner::Point GlobalPlanner::generate_next_goal(){

      Point nxt_point = {};

      bool point_found = true;

      while(true) {

        __uint32_t mx_ , my_, mx_i, my_i;
        
        mx_= rand() % (map_xf - map_xi + 1) + (map_xi);
        my_= rand() % (map_yf - map_yi + 1) + (map_yi);
       
        if(mx_ >= map_xf || mx_ <= map_xi || my_ >= map_yf || my_ <= map_yi) {

          cout << "out of bound point found! ---- SKIPPING" << endl;
          continue;

        }

        int is_reachable = check_cell_neighbour(Point{mx_, my_});

        if(is_reachable == 0) {continue;}
        
        nxt_point = Point{mx_, my_};
        return nxt_point;
        
      }

      cout << "Something is wrong --- generate_next_goal function!" << endl;
      cout << "Sleeping for 5 seconds!" << endl;
      ros::Duration(5.0).sleep();

      return Point{};       
  
  }

  bool GlobalPlanner::is_point_reachable(const Point &p1, const Point &p2, int dis) {

    //double dis = heu(p1, p2);

    int step_sz = 1;

    int num_steps = (dis/ step_sz) + 1;

    int mx_c = p1.x , my_c = p1.y;    

    double ang_ = atan2((int)p2.y - (int)p1.y, (int)p2.x - (int)p1.x);

    int is_reachable = 1;

    while(num_steps > 0) {

        __uint32_t mx_d = mx_c + num_steps * step_sz * cos(ang_);
        __uint32_t my_d = my_c + num_steps * step_sz * sin(ang_);

        is_reachable = check_cell_neighbour(Point{mx_d, my_d});

        if(!is_reachable) {break;}
        
        num_steps--;  

        
    }

    return is_reachable;

  }

  GlobalPlanner::rrt_star_cell* GlobalPlanner::get_closest_cell(const GlobalPlanner::Point &nxt_pt, int dis_r) {
      
      long double mn_dis = std::numeric_limits<int>::max();  
      rrt_star_cell* best_cell = nullptr;

      for(int i = 0; i < (int)rrt_tree.size(); i++) {

        double dis = heu(nxt_pt, rrt_tree[i]->point);

        int dis_ = min(dis_r, (int)dis);

        bool is_reachable = is_point_reachable(nxt_pt, rrt_tree[i]->point, dis_);

        if(!is_reachable){continue;}

        if(dis < mn_dis) {

          mn_dis = dis;
          best_cell = rrt_tree[i];

        }

      }
      
      if(best_cell == nullptr) {

        cout << "Something might be wrong --- get_closest cell returns nullptr"<< endl;
        //ros::Duration(4.0).sleep();

      }

      return best_cell;
      
  }


  void GlobalPlanner::delete_all_markers() {

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = nh_.getNamespace();

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::DELETEALL;

    
    marker.id = marker_id_cnt++;
    marker.header.stamp = ros::Time();

    double wx_, wy_; 

    goal_marker_pub.publish( marker);   

  }


  void GlobalPlanner::publish_marker_point(const Point &curr_pt, int flag, int duration) {

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
    marker.ns = nh_.getNamespace();

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.color.a = 1.0; // Don't forget to set the alpha!
    
    marker.id = marker_id_cnt++;
    marker.header.stamp = ros::Time();

    double wx_, wy_; 
    costmap_ros_->mapToWorld(curr_pt.x, curr_pt.y, wx_, wy_);

    marker.pose.position.x = wx_;
    marker.pose.position.y = wy_;
    marker.pose.position.z = 1;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;

    if(duration == 0) {marker.lifetime = ros::Duration();}      

    else {marker.lifetime = ros::Duration(duration);}

    if(flag == 1) {

      marker.color = green; 
      //marker.lifetime = ros::Duration();

    }

    if(flag == 0) {

      marker.color = red;
     
    }

    if(flag == -1) {

      marker.color = blue; 
     
    }
    goal_marker_pub.publish( marker);   

  }


    
  void GlobalPlanner::update_rrt_star_path_points(vector<Point> &path_points, rrt_star_cell* final_cell){

    rrt_star_cell* curr_cell = final_cell; 

    while(curr_cell != nullptr){

      path_points.push_back(curr_cell->point);
      curr_cell = curr_cell->parent;
      //publish_marker_point(curr_cell->point, 1, 0);
    }

  }
  
  void GlobalPlanner::update_rrt_star_planner_plan(vector<geometry_msgs::PoseStamped> &plan, const geometry_msgs::PoseStamped &goal, const vector<Point> &path_points) {


    for(int i = 0; i < (int)path_points.size(); i++){

      geometry_msgs::PoseStamped curr_pt = goal;

      double wx_c, wy_c; 
      __uint32_t mx_c = path_points[i].x, my_c = path_points[i].y;

      costmap_ros_->mapToWorld(mx_c, my_c, wx_c, wy_c);

      curr_pt.pose.position.x = wx_c;
      curr_pt.pose.position.y = wy_c;      

      plan.push_back(curr_pt);

    }

  }

  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
    
    bool reached = false;
    rrt_tree.clear(); 
    
    __uint32_t mx_i, my_i, mx_f, my_f, mx_c, my_c;

    print_world_params(start, goal, mx_i, my_i, mx_f, my_f);

    mx_c = mx_i, my_c = my_i;

    rrt_star_cell* head_cell = new rrt_star_cell();
    head_cell->point = Point{mx_i, my_i};
    head_cell->parent = nullptr;

    rrt_tree.push_back(head_cell);

    vector<Point> path_points;
    path_points.clear(); 

    delete_all_markers();

    int cnt =0 ;

    while(true){
      
      int dis_r = 60;

      Point nxt_pt = generate_next_goal();

      publish_marker_point(nxt_pt, -1, 1);

      rrt_star_cell* best_cell = get_closest_cell(nxt_pt, dis_r);

      if(best_cell == nullptr) {continue;}

      //publish_marker_point(best_cell->point, 0);

      rrt_star_cell* best_goal_cell = get_closest_cell(Point{mx_f, my_f}, dis_r);

      double dis_from_best_cell = heu(best_cell->point, nxt_pt);
      double dis_from_goal = heu(Point{mx_f, my_c}, best_goal_cell->point);

      if(best_goal_cell != nullptr && dis_from_goal < 20) {
        
        reached = true; 

        rrt_star_cell* final_cell = new rrt_star_cell(); 
        final_cell->parent = best_goal_cell;
        final_cell->point = Point{mx_f, my_f};
        rrt_tree.push_back(final_cell);
        

        update_rrt_star_path_points(path_points, final_cell);
        reverse(path_points.begin(), path_points.end());

        update_rrt_star_planner_plan(plan, goal, path_points);
        publish_global_path(plan, goal);
        
        for(int i =0; i < (int)path_points.size(); i++){
          
          publish_marker_point(path_points[i], 1, 0);

        }

        cout << "Almost reached the goal!" << endl;
        ros::Duration(3.0).sleep();

        return true;

      }

      
      //int dis_r = 60; 

      dis_r = min(dis_r, int(dis_from_best_cell));
      
      int step_sz = 1;

      int num_steps = (dis_r/step_sz) + 1;

      mx_c = best_cell->point.x ,my_c = best_cell->point.y;

      double ang_ = atan2((int)nxt_pt.y - (int)my_c, (int)nxt_pt.x - (int)mx_c);

      double cos_ =  cos(ang_);
      double sin_ = sin(ang_);

      //double ang_ = atan2(nxt_pt.y - (int)my_c, nxt_pt.x - (int)mx_c);

      cout <<"(nxt_pt.x,nxt_pt.y): (" << nxt_pt.x <<","<<nxt_pt.y <<")" << endl;
      cout << "(mx_c,my_c): (" << mx_c  << "," << my_c  <<")" << endl;
      
      //ros::Duration(1.0).sleep();

      bool flag = (mx_c > nxt_pt.x);

      
      //cout << "(mx_c, mx_c): (" << mx_c << "," << my_c <<")" << endl;

      bool is_reachable = is_point_reachable(Point{mx_c, my_c}, nxt_pt, dis_r);

      if(!is_reachable) { continue; }

      __uint32_t old_mxc = mx_c, old_myc = my_c;

      mx_c = (int)mx_c + ((int)num_steps * step_sz * cos_);
      my_c = (int)my_c + ((int)num_steps * step_sz * sin_);

      rrt_star_cell* latest_cell = new rrt_star_cell(); 
      latest_cell->point = Point{mx_c, my_c};
      latest_cell->parent = best_cell;

      publish_marker_point(Point{mx_c, my_c}, -1, 0);

      
      rrt_tree.push_back(latest_cell);

    }
  
  
  }

};
