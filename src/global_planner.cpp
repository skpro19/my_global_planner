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


  GlobalPlanner::RRT_Cell* GlobalPlanner::get_closest_cell(const Point &nxt_pt) {
      
      if(head_cell == nullptr) {return nullptr;} 

      if(head_cell->children.size() == 0) {

        return head_cell;

      }

      long double mn_dis = heu(nxt_pt, head_cell->point); 
      RRT_Cell* best_cell = head_cell;

      for(int i = 0; i < (int)head_cell->children.size(); i++) {

        RRT_Cell* curr_cell = get_closest_cell(nxt_pt, head_cell->children[i]);

        double dis = heu(nxt_pt, curr_cell->point);

        if(dis < mn_dis) {

          mn_dis = dis;
          best_cell = curr_cell;

        }

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

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = nh_.getNamespace();

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.lifetime = (flag==1) ? ros::Duration() : ros::Duration(3.0);

    marker.scale.x = (flag == 1) ? 0.5 : 0.25;
    marker.scale.y = (flag == 1) ? 0.5 : 0.25;
    marker.scale.z = (flag == 1) ? 0.5 : 0.25;
    
    marker.color.a = 1.0; // Don't forget to set the alpha!
    
    marker.color = (flag == 1) ? green: blue;

    marker.id = marker_id_cnt++;
    marker.header.stamp = ros::Time();

    double wx_, wy_; 
    costmap_ros_->mapToWorld(curr_pt.x, curr_pt.y, wx_, wy_);

    marker.pose.position.x = wx_;
    marker.pose.position.y = wy_;
    marker.pose.position.z = 1;


    goal_marker_pub.publish( marker);   

  }

  void GlobalPlanner::update_RRT_path_points(vector<Point> &path_points, RRT_Cell *final_cell) {
    
    int cnt =0;

    while(final_cell != nullptr) {
      
      Point curr_pt = final_cell->point;
    
      publish_marker_point(curr_pt, 1);
      
      path_points.push_back(curr_pt);
       
    
      final_cell = final_cell->parent;
    
    }

    
  }

  void GlobalPlanner::update_RRT_planner_plan(vector<geometry_msgs::PoseStamped> &plan, const geometry_msgs::PoseStamped &goal, const vector<Point> &path_points) {


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

  bool GlobalPlanner::check_cell_neighbour(const Point &pt) {
    
    int margin_sz = 5;

    for(int i = (int)pt.x - margin_sz; i < (int)pt.x + margin_sz; i++) {

      for(int j = (int)pt.y - margin_sz; j <= (int)pt.y + margin_sz; j++) {
        
        if(i <= map_xi || j <= map_xi || i >= map_xf || j>= map_yf) {return false;}

        unsigned char cell_cost = costmap_ros_->getCost(i,j);

        if(cell_cost == costmap_2d::LETHAL_OBSTACLE) {return false;}

      }

    }

    return true;


  }

  bool GlobalPlanner::is_point_reachable(const Point &best_pt, const Point &nxt_pt, __uint32_t step_sz, __uint32_t &mx_c , __uint32_t &my_c){

      //__uint32_t step_sz = 20;

      double dis = heu(best_pt, nxt_pt);

      if((__uint32_t) dis < step_sz) {

        step_sz = (__uint32_t)dis;

      }

      bool valid_pt= true;

      //double dis = heu(nxt_pt, Point{best_pt.x, best_pt.y});

      double sin_th = double((int)nxt_pt.y - (int)best_pt.y) / dis;
      double cos_th = double((int)nxt_pt.x - (int)best_pt.x) / dis;
      

      for(int i = 0; i <= step_sz; i++) {

        __uint32_t mx_d = (int)best_pt.x + i * cos_th;
        __uint32_t my_d = (int)best_pt.y+ i* sin_th;

        if(mx_d <= map_xi || mx_d >= map_xf || my_d <= map_yi || my_d >= map_yf) {

          valid_pt = false;
          break;

        }

        bool flag = check_cell_neighbour(Point{mx_d, my_d});

        if(!flag){

          valid_pt = false;
          break;
          
        }

      }

      if(valid_pt){


        mx_c = best_pt.x + step_sz * cos_th;
        my_c = best_pt.y + step_sz * sin_th;


      }

      return valid_pt;

  }


  bool GlobalPlanner::is_point_reachable(const Point &best_pt, const Point &nxt_pt, __uint32_t step_sz, __uint32_t &mx_c , __uint32_t &my_c){

      //__uint32_t step_sz = 20;

      double dis = heu(best_pt, nxt_pt);

      if((__uint32_t) dis < step_sz) {

        step_sz = (__uint32_t)dis;

      }

      bool valid_pt= true;

      //double dis = heu(nxt_pt, Point{best_pt.x, best_pt.y});

      double sin_th = double((int)nxt_pt.y - (int)best_pt.y) / dis;
      double cos_th = double((int)nxt_pt.x - (int)best_pt.x) / dis;
      

      for(int i = 0; i <= step_sz; i++) {

        __uint32_t mx_d = (int)best_pt.x + i * cos_th;
        __uint32_t my_d = (int)best_pt.y+ i* sin_th;

        if(mx_d <= map_xi || mx_d >= map_xf || my_d <= map_yi || my_d >= map_yf) {

          valid_pt = false;
          break;

        }

        bool flag = check_cell_neighbour(Point{mx_d, my_d});

        if(!flag){

          valid_pt = false;
          break;
          
        }

      }

      if(valid_pt){


        mx_c = best_pt.x + step_sz * cos_th;
        my_c = best_pt.y + step_sz * sin_th;


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

    vector<Point> path_points;

    while(true) {

      Point nxt_pt = generate_next_goal();
      //last_marker_id_cnt = marker_id_cnt;

      
      rrt_star_cell* best_cell = get_closest_cell(nxt_pt);
      Point best_pt = best_cell->point;
      
      rrt_star_cell* best_goal_cell = get_closest_cell(Point{mx_f, my_f});

      //double dis_from_goal = heu(nxt_pt, Point{mx_f, my_f});
      double dis_from_goal = heu(Point{mx_f, my_f}, best_goal_cell->point);

      if(dis_from_goal < 6) {
        
        
        cout << "Almost reached the goal!" <<endl;
        cout <<"dis_from_goal: " << dis_from_goal << endl;
        //cout <<"Sleeping for 2 seconds!" << endl;
        //ros::Duration(2.0).sleep();

        if(is_point_reachable(best_goal_cell->point, Point{mx_f, my_f}, (__uint32_t)dis_from_goal, mx_c, my_c)) {

          rrt_star_cell* final_cell = new rrt_star_cell();
          final_cell->point = Point{mx_f, my_f};
          final_cell->parent = best_goal_cell;  
          //best_goal_cell->children.push_back(final_cell);

          update_rrt_star_path_points(path_points, final_cell);
          reverse(path_points.begin(), path_points.end());
          update_rrt_star_planner_plan(plan, goal, path_points);
          publish_global_path(plan, goal);


          reached = true;
          break;

        }

        else {

          continue;

        }        

        
      }
      
      __uint32_t step_sz = 100;

      bool valid_pt = is_point_reachable(best_pt, nxt_pt, step_sz, mx_c, my_c);
      
      if(!valid_pt) {continue;}
      
      rrt_star_cell* latest_cell = new rrt_star_cell();
      latest_cell->point = Point{mx_c, my_c};
      latest_cell->parent = best_cell;
      
      //best_cell->children.push_back(latest_cell);

    }


    cout <<"reached: " << reached << endl;


    if(!reached) {

      cout << "Something is wrong! ---- Could not reach near the goal!" << endl;

    }

    return true;

  }

};
