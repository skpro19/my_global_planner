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
    cout << "Sleeping for 3 seconds!" << endl;

    ros::Duration(3.0).sleep();

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

        unsigned char cell_cost = costmap_ros_->getCost(mx_, my_);
        
        bool flag = check_cell_neighbour(Point{mx_, my_});

        if(!flag) {continue;}

        publish_marker_point(Point{mx_, my_}, -1);
        
        nxt_point = Point{mx_, my_};
        return nxt_point;
        
      }

      cout << "Something is wrong --- generate_next_goal function!" << endl;
      cout << "Sleeping for 5 seconds!" << endl;
      ros::Duration(5.0).sleep();

      return Point{};       
  
  }


  GlobalPlanner::rrt_star_cell* GlobalPlanner::get_closest_cell(const Point &nxt_pt) {
      
      double mn_dis = std::numeric_limits<double>::max();

      rrt_star_cell* best_cell= nullptr;

      for(int i= 0; i < (int)rrt_tree.size(); i++){

        double dis = heu(nxt_pt, rrt_tree[i]->point);

        if(dis < mn_dis){

          mn_dis = dis; 
          best_cell = rrt_tree[i];

        }

      }

      if(best_cell == nullptr){

        cout <<"Something is wrong -- Inside the get_closest_cell function --- Sleeping for 5 seconds!" << endl;
        ros::Duration(5.0).sleep();

      }

      return best_cell;
      
  }

  void GlobalPlanner::publish_marker_point(const Point &curr_pt, int flag) {

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

    std_msgs::ColorRGBA col_arr[10];

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = nh_.getNamespace();

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.lifetime = ros::Duration();

    marker.scale.x = 0.5;
    marker.scale.y = 0.5 ;
    marker.scale.z = 0.5;

    if(flag == -1) {

      marker.scale.x = 0.25; 
      marker.scale.y = 0.25;
      marker.lifetime = ros::Duration(3.0);
      marker.color = blue;

    }

    if(flag == 0){

      marker.scale.x = 1; 
      marker.scale.y = 1;
      marker.lifetime = ros::Duration(1.0);
      marker.color = red;
    }

    if(flag == 1) {

      marker.scale.x = 0.5; 
      marker.scale.y = 0.5;
      marker.lifetime = ros::Duration();
      marker.color = green;
      
    }

    marker.color.a = 1.0; // Don't forget to set the alpha!
    

    marker.id = marker_id_cnt++;
    marker.header.stamp = ros::Time();

    double wx_, wy_; 
    costmap_ros_->mapToWorld(curr_pt.x, curr_pt.y, wx_, wy_);

    marker.pose.position.x = wx_;
    marker.pose.position.y = wy_;
    marker.pose.position.z = 1;

    goal_marker_pub.publish( marker);   

  }

  void GlobalPlanner::update_rrt_star_path_points(vector<Point> &path_points, rrt_star_cell *final_cell) {
    
    int cnt =0;

    while(final_cell != nullptr) {
      
      
      Point curr_pt = final_cell->point;
      cout << "curr_pt.x: " << curr_pt.x << " curr_pt.y: " << curr_pt.y << endl;
      cnt++;
      cout <<"cnt: " << cnt << endl << endl;
      publish_marker_point(curr_pt, 1);
      
      path_points.push_back(curr_pt);
       
    
      final_cell = final_cell->parent;
    
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

  bool GlobalPlanner::check_cell_neighbour(const Point &pt, int margin_sz) {
    
    
    for(int i = (int)pt.x - margin_sz; i < (int)pt.x + margin_sz; i++) {

      for(int j = (int)pt.y - margin_sz; j <= (int)pt.y + margin_sz; j++) {
        
        if(i <= map_xi || j <= map_xi || i >= map_xf || j>= map_yf) {return false;}

        unsigned char cell_cost = costmap_ros_->getCost(i,j);

        if(cell_cost == costmap_2d::LETHAL_OBSTACLE) {return false;}

      }

    }

    return true;


  }

  bool GlobalPlanner::is_point_reachable(const Point &best_pt, const Point &nxt_pt, __uint32_t dis_r){

      __uint32_t step_sz = 1;

      double dis = heu(best_pt, nxt_pt);

      if(dis < step_sz) {

        cout << "Possible error! ----  num_steps inside the is_point_reachable function is 0!" << endl;
        //cout << "Sleeping for 3 seconds!" << endl;
        //ros::Duration(3.0).sleep();

        return true;

      }

      bool valid_pt= true;

      int num_steps = (dis_r / step_sz) + 1 ;

      double ang_ = atan2(nxt_pt.y - best_pt.y , nxt_pt.x - best_pt.x);

      for(int i = 0; i <= num_steps; i++) {

        __uint32_t mx_d = (int)best_pt.x + i * step_sz* cos(ang_);
        __uint32_t my_d = (int)best_pt.y+ i* step_sz* sin(ang_);
        
        bool flag = check_cell_neighbour(Point{mx_d, my_d});

        if(!flag){

          valid_pt = false;
          break;
          
        }

      }

      return valid_pt;

  }

  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    bool reached = false;

    __uint32_t mx_i, my_i, mx_f, my_f, mx_c, my_c;

    print_world_params(start, goal, mx_i, my_i, mx_f, my_f);

    mx_c = mx_i, my_c = my_i;

    rrt_star_cell* head_cell = new rrt_star_cell();
    head_cell->point = Point{mx_i, my_i};
    head_cell->parent = nullptr;

    rrt_tree.push_back(head_cell);

    vector<Point> path_points;

    while(true) {

      Point nxt_pt = generate_next_goal();

      rrt_star_cell* best_cell = get_closest_cell(nxt_pt); 
      rrt_star_cell* best_goal_cell = get_closest_cell(Point{mx_f, my_f});

      double dis_from_goal = heu(Point{mx_f, my_f}, best_goal_cell->point);

      if(dis_from_goal < 20) {

        bool poss_ = is_point_reachable(best_goal_cell->point, Point{mx_f, my_f}, dis_from_goal);

        if(!poss_) {continue;}

        rrt_star_cell* final_cell = new rrt_star_cell();
        final_cell->point = Point{mx_f, my_f};
        final_cell->parent = best_goal_cell;

        update_rrt_star_path_points(path_points, final_cell);
        reverse(path_points.begin(), path_points.end());
        update_rrt_star_planner_plan(plan, goal, path_points);
        publish_global_path(plan,goal);

        cout << "Almost reached the goal --- Sleeping for 3 seconds!" << endl;
        ros::Duration(3.0).sleep();
      
        return true;

      }

      int dis_r = 60; 

      double dis_ = heu(best_cell->point, nxt_pt);

      //dis_r = min(dis_r , (int)dis_ + 1);

      //bool reachable_ = is_point_reachable(best_cell->point, nxt_pt, dis_r);

      bool reachable_ = true;

      int step_sz = 1; 

      int num_steps = dis_r/step_sz + 1; 

      double ang_ = atan2(nxt_pt.y - best_cell->point.y, nxt_pt.x - best_cell->point.x);

      while(num_steps > 0) {

        int mx_c = best_cell->point.x + num_steps * dis_r * cos(ang_);
        int my_c = best_cell->point.y + num_steps * dis_r * sin(ang_);

        reachable_ = check_cell_neighbour(Point{(__uint32_t)mx_c, (__uint32_t)my_c});

        if(!reachable_) {break;}

        num_steps--;
      
      }

      if(!reachable_) {continue;}

      Point latest_pt = {best_cell->point.x + int(dis_r * cos(ang_)) , best_cell->point.y + int(dis_r * sin(ang_))}; 

      publish_marker_point(latest_pt, 1);
      rrt_star_cell* latest_cell = new rrt_star_cell();
      latest_cell->point = latest_pt; 
      latest_cell->parent = best_cell;
      ros::Duration(1.0).sleep();
      rrt_tree.push_back(latest_cell);
    
    }


  }

};   
