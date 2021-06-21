/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <map>
#include<nav_msgs/Path.h>


#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP



namespace global_planner {

    class GlobalPlanner : public nav_core::BaseGlobalPlanner {
        
        
        struct Point {
            
                __uint32_t x, y; 

                bool operator==(const Point &p1 ) const{   return ((p1.x == x) && (p1.y == y));  }   
                
                bool operator<(const Point &p1 ) const{    return ((p1.x < x) || (p1.x == x && p1.y < y) ) ;  }   

            };

        
        struct Cell {

            Point point; 
            __uint32_t cost_till_now;

            bool operator<(const Cell &c1) const {
                
                
                return (c1.cost_till_now < cost_till_now || (c1.cost_till_now == cost_till_now && c1.point < point));

            }
        
        };

        struct RRT_Cell{

            Point point;
            RRT_Cell* parent;
            std::vector<RRT_Cell*> children;
            
        };



        public:

            GlobalPlanner();
            
            GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);

            

        private: 


            double heu(Point p1, Point p2);
            void update_planner_plan(std::vector<Point> &path_points, std::vector<geometry_msgs::PoseStamped> &plan, const geometry_msgs::PoseStamped &goal); 
            void publish_global_path(const std::vector<geometry_msgs::PoseStamped> &plan, const geometry_msgs::PoseStamped &goal);
            bool print_cell(const Cell &cell);
            void print_world_params(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, __uint32_t &mx_i, __uint32_t &my_i, __uint32_t &mx_f, __uint32_t &my_f);
            Point generate_next_goal();
            void update_map_bounds();
            RRT_Cell* get_closest_cell(const Point &nxt_pt, RRT_Cell* head_cell);
            bool RRT_path_so_far(RRT_Cell *head);
            void publish_marker_point(const Point &pt, int flag);
            void update_RRT_path_points(std::vector<Point> &path_points, RRT_Cell* final_cell);
            void update_RRT_planner_plan(std::vector<geometry_msgs::PoseStamped> &plan, const geometry_msgs::PoseStamped &goal, const std::vector<Point> &path_points);
            bool check_cell_neighbour(const Point &pt);
            bool is_point_reachable(const Point &last_point, const Point &curr_point, __uint32_t step_sz, __uint32_t &mx_c, __uint32_t &my_c);



            costmap_2d::Costmap2D* costmap_ros_;
            costmap_2d::Costmap2DROS *my_costmap_ros;
            __uint32_t size_x, size_y;
            __uint32_t map_xi, map_xf, map_yi, map_yf;
            ros::Publisher global_plan_pub, goal_marker_pub;
            ros::Subscriber pose_sub;
            ros::NodeHandle nh_;
            int marker_id_cnt;
    };

};

#endif