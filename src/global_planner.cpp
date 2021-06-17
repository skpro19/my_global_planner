#include <pluginlib/class_list_macros.h>
#include <my_global_planner/global_planner.h>
#include <tf/tf.h>
#include <queue>
#include<math.h>
#include <cstdlib>
#include <ctime>
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

    cout << "Inside the make_plan function!" << endl;
    cout << "Hello World!" << endl;
    
    //__uint32_t mx_i, my_i, mx_f, my_f;

    double wx_i, wy_i, wx_f, wy_f;

    cout << "(int)costmap_2d::LETHAL_OBSTACLE: " << (int)costmap_2d::LETHAL_OBSTACLE << endl;
    cout << "(int)costmap_2d::NO_INFORMATION: " << (int)costmap_2d::NO_INFORMATION << endl;
    cout << "(int)costmap_2d::FREE_SPACE: " << (int)costmap_2d::FREE_SPACE << endl; 

    //cout << "Sleeping for 2 seconds!" << endl;
    //ros::Duration(2.0).sleep();

    /*cout << "Testing costmap_ros for consistency!" << endl;

    cout << "ros_costmap->getGlobalFrameID(): " << costmap_ros->getGlobalFrameID() << endl;
    cout << "Sleeping for 2 seconds!" << endl;
    ros::Duration(2.0).sleep();

    cout << "Trying to fetch global_pose of the robot!" << endl;
    geometry_msgs::PoseStamped global_pose_;
    costmap_ros->getRobotPose(global_pose_);

    cout << "global_pose.x: " << global_pose_.pose.position.x << " global_pose_.y: " << global_pose_.pose.position.y  << endl;
    */


    cout << "Sleeping for 2 seconds!" << endl;
    ros::Duration(2.0).sleep();
    
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

  }

  bool GlobalPlanner::makePlanOne(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    //Djikstra's Algorithm

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

    map<Point, __uint32_t> cell_cost_index;
    map<Point, Point> from_index;

    Cell start_cell  = {Point{mx_i, my_i}, 0};

    Point last_point;

    priority_queue<Cell> pq;

    bool reached = false;

    int cnt = 0 ;

    pq.push(start_cell);

    while(!pq.empty()) {
  
      Cell top_cell = pq.top(); 
      Point top_point = top_cell.point;

      pq.pop();
      
      if(top_point == Point{mx_f, my_f}) {

        cout << "Reached the goal!" << endl;  
        last_point = top_point;
        reached = true;
        break;
        
      }

      __uint32_t mx_top = top_point.x , my_top = top_point.y;

      Point best_pt = {}; 
      __uint32_t mn_cost = UINT32_MAX;

      for(int i = (int)mx_top - 1; i <= (int)mx_top + 1; i++) {

        for(int j= (int)my_top - 1; j <= (int)my_top + 1; j++) {

          if(i == mx_top && j == my_top) {continue;}

          if(i < 0 || j < 0 || i >= size_x || j >= size_y) {continue;}

          
          Point nxt_point = Point{i, j};

          __uint32_t curr_cell_cost = (__uint32_t)costmap_ros_->getCost(nxt_point.x, nxt_point.y);


          if(cell_cost_index.find(nxt_point) == cell_cost_index.end()) { cell_cost_index[nxt_point] = UINT32_MAX; }
        
          __uint32_t new_cost = top_cell.cost_till_now + curr_cell_cost + 1;
        
          if(new_cost < cell_cost_index[nxt_point]) {
            
            cell_cost_index[nxt_point] = new_cost;

            __uint32_t priority = new_cost ;
            //__uint32_t priority = new_cost + heu(nxt_point, Point{goal.pose.position.x, goal.pose.position.y});
            
            cout << "Pushing (" << nxt_point.x <<"," << nxt_point.y <<") into the pq!" << endl;
            
            pq.push(Cell{nxt_point, priority});

            from_index[nxt_point] = top_point;
          
          }

        }

      }

    }
    



    cout << "After the while loop!" << endl;
    cout << "pq.size(): " <<pq.size() << endl;
    cout << "cnt: " << cnt << endl;
    cout << "REACHED : " << reached << endl;

    cout << "Updating planner path!" << endl;
    cout << "Sleeping for 5 seconds!" << endl;
    ros::Duration(5.0).sleep();

    if(!reached) {return true;}

    vector<Point> my_path_points;

    while(true){

        my_path_points.push_back(last_point);
        
        if(last_point == start_cell.point) {break;}

        last_point = from_index[last_point];

    }

    //my_path_points.push_back(last_point);

    cout << "(int)my_path_points.size(): " << my_path_points.size() << endl;

    reverse(my_path_points.begin(), my_path_points.end());
   
    update_planner_plan(my_path_points, plan, goal);    

    publish_global_path(plan, goal);
    
    return true;

  }


  GlobalPlanner::Point GlobalPlanner::generate_next_goal(){

      

      cout << "Inside the generate_next_goal function!" << endl;
      cout << "Sleeping for 2 seconds!" << endl; 

      ros::Duration(2.0).sleep();

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
        
        cout << "Getting pose in the global frame!" << endl;


        __uint32_t mx_ , my_, mx_i, my_i;
        double wx_, wy_, wx_i, wy_i;

        geometry_msgs::PoseStamped global_pose_;
        my_costmap_ros->getRobotPose(global_pose_);

        wx_i = global_pose_.pose.position.x , wy_i = global_pose_.pose.position.y;
        costmap_ros_->worldToMap(wx_i, wy_i, mx_i, my_i);
                
        mx_ = rand()%((int)map_xf - (int)map_xi + 1) + map_xi;
        my_ = rand()% ((int)map_yf - (int)map_yi + 1) + map_yi;

        cout  << "mx_: " << mx_ << " my_: " << my_ << endl;

        costmap_ros_->mapToWorld(mx_, my_, wx_, wy_);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = nh_.getNamespace();

        marker.id = marker_id_cnt++;

        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = wx_;
        marker.pose.position.y = wy_;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 0.5;
        
        marker.color.a = 1.0; // Don't forget to set the alpha!
        
        marker.color = green;

        unsigned char cell_cost = costmap_ros_->getCost(mx_, my_);


        if(cell_cost == costmap_2d::LETHAL_OBSTACLE) { 
        
          marker.color = red;
          marker.scale.x = 1; 
          marker.scale.y = 1;

        }


        double dis = heu(Point{mx_, my_}, Point{mx_i, my_i});
        
        if(dis < 10) {
          
          marker.color = blue;
          marker.scale.x = 2;
          marker.scale.y = 2;

        }

        
        goal_marker_pub.publish( marker );        

        if(cell_cost != costmap_2d::LETHAL_OBSTACLE && dis > 10) {

          point_found = true;    
          nxt_point = Point{mx_, my_};
          cout << "Found next point: (:" << nxt_point.x <<"," << nxt_point.y <<")" << endl;
          
          cout << "Sleep for 1 second!" << endl;
          ros::Duration(1.0).sleep();
          break;
          
        }


      }

      if(!point_found) {

        cout << "No suitable point found ---- ERROR!" << endl;
        cout << "Sleeping for 5 seconds!" << endl;
        ros::Duration(5.0);      
      }

      return nxt_point;      
  
  }

  GlobalPlanner::RRT_Cell* GlobalPlanner::get_next_best_cell(const Point &curr_point, RRT_Cell* head_cell){

      RRT_Cell* curr_cell = head_cell;

      double mn_dis = UINT32_MAX; 
      RRT_Cell* best_cell;


      while(true) {

        if(curr_cell == NULL) {
          
          cout << "best_cell->point.x: " << best_cell->point.x << " best_cell->point.y: "<< best_cell->point.y << endl;
          break;

        }

        Point pt = curr_cell->point;
        double dis = heu(pt, curr_point);

        if(dis < mn_dis) {

          mn_dis = dis;
          best_cell = curr_cell;

        }

        curr_cell = curr_cell->next_cell;

      }
      
      return best_cell;


  }

  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    __uint32_t mx_i, my_i, mx_f, my_f;

    print_world_params(start, goal, mx_i, my_i, mx_f, my_f);

    RRT_Cell* head_cell;
    head_cell->point = Point{mx_i, my_i};
    head_cell->next_cell=NULL;
    head_cell->prev_cell=NULL;

    while(true){

      Point p = generate_next_goal();

      cout << "Sleeping for 1 second!" << endl;
      ros::Duration(1.0).sleep();

      RRT_Cell* best_cell = get_next_best_cell(p, head_cell);
      
      cout << "best_cell->point.x: " << best_cell->point.x << " best_cell->point.y: " << best_cell->point.y << endl;
      cout << "Sleeping for 1 second!" << endl;
      ros::Duration(1.0).sleep();

      


    }

    return true;

  }
  
  

};
