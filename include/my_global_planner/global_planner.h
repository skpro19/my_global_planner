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

            bool operator==(const Point &p1 ) {    return ((p1.x == this->x) && (p1.y == this->y));  }   
            
            bool operator<(const Point &p1 ) const{    return (p1.x < this->x) ;  }   


        };


       


        struct Cell {

            Point point; 
            __uint32_t cost_till_now;

        };


        struct compare_cost{
            
            bool operator() (Cell &p1, Cell &p2) 
            {

                return p1.cost_till_now < p2.cost_till_now;
            }

        };

        public:

            GlobalPlanner();
            GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            bool makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan
                        );

            bool makePlanOne(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan );
            bool makePlanTwo(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan );
            bool makePlanThree(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan );
            bool make_straight_plan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan);


        private: 


            double heu(Point p1, Point p2);


            costmap_2d::Costmap2D* costmap_ros_;
            costmap_2d::Costmap2DROS *costmap_ros;
            __uint32_t size_x, size_y;
            ros::Publisher global_plan_pub;
            
    };

};

#endif