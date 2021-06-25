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

        
        struct rrt_star_cell{

            Point point;
            rrt_star_cell* parent;
            __uint32_t cost_till_now;
            int id;
        
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
            void print_world_params(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, __uint32_t &mx_i, __uint32_t &my_i, __uint32_t &mx_f, __uint32_t &my_f);
            Point generate_next_goal();
            void update_map_bounds();
            rrt_star_cell* get_closest_cell(const Point &nxt_pt);
            void publish_marker_point(const Point &pt, int flag);
            void update_rrt_star_path_points(std::vector<Point> &path_points, rrt_star_cell *final_cell);
            void update_rrt_star_planner_plan(std::vector<geometry_msgs::PoseStamped> &plan, const geometry_msgs::PoseStamped &goal, const std::vector<Point> &path_points);
            bool check_cell_neighbour(const Point &pt, int margin_sz = 5);
            bool is_point_reachable(const Point &last_point, const Point &curr_point, __uint32_t dis_r);
            bool add_cell_to_tree(rrt_star_cell* curr_cell, __uint32_t search_r);
            bool is_point_reachable_with_cost(const Point &best_pt, const Point &nxt_pt, __uint32_t dis_r, __uint32_t &cost);
            void update_tree_connections(rrt_star_cell* curr_cell, __uint32_t search_r);
            __uint32_t get_cost_between_points(const Point &p1 , const Point &p2);
            bool connect_node_to_tree(rrt_star_cell* final_cell, int search_r);
            bool is_point_reachable(const Point &best_pt, const Point &nxt_pt, __uint32_t step_sz, __uint32_t &mx_c , __uint32_t &my_c);


            std::vector<rrt_star_cell*> rrt_tree;
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