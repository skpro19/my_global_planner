#include <pluginlib/class_list_macros.h>
#include <my_global_planner/global_planner.h>
#include <tf/tf.h>

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

    costmap_ros_ = costmap_ros->getCostmap(); 

    cout << "global_frame: " << costmap_ros->getGlobalFrameID() << endl;

 }

 bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){


   string frame_id = start.header.frame_id; 

   cout << "frame_id: " << frame_id << endl; 

   if(frame_id != "map") {

     cout << "Start Pose is not in map frame! ---- ABORTING" << endl;
     return false;

   }

    double wx_i = start.pose.position.x, wy_i = start.pose.position.y;
    double wx_f = goal.pose.position.x, wy_f = goal.pose.position.y;


    __uint32_t mx_i, my_i, mx_f, my_f;

    costmap_ros_->worldToMap(wx_i, wy_i, mx_i, my_i);

    costmap_ros_->worldToMap(wx_f, wy_f, mx_f, my_f);

    __uint32_t mx_c = mx_i, my_c = my_i;

    plan.push_back(start);
    
    while(my_c  < my_f) {

      my_c++;

      double wx_c, wy_c;

      costmap_ros_->mapToWorld(mx_c, my_c, wx_c, wy_c);

      geometry_msgs::PoseStamped next_pose = start;

      next_pose.pose.position.x = wx_c; 
      next_pose.pose.position.y = wy_c;
      
      plan.push_back(next_pose);

    } 

    cout << "Size of plan: " << (int)plan.size() << endl;

    return true;

  
 }


 };