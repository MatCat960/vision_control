#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <cmath>


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <vision_control/VisionController.h>
#include <tf2/utils.h>

#define M_PI   3.14159265358979323846  /*pi*/

class myNode
{
private:
    ros::Publisher pub;
    ros::Publisher communication_pub;
    ros::Subscriber sub;
    ros::Subscriber pose_sub;
    ros::Subscriber neighbors_sub;
    ros::Subscriber target_sub;
    ros::Subscriber obstacles_sub;
    ros::NodeHandle n, nh_;
    ros::Timer timer;
    vision_control::VisionController controller;
    // std::vector<Eigen::Vector2d> mates, enemies;
    Eigen::Vector3d ustar,p_i;
    Eigen::VectorXd h;
    Eigen::MatrixXd p_js_i;                 // local
    Eigen::Vector2d p_target;               // global
    Eigen::MatrixXd obs, obs_i;                  // global, local
    Eigen::Vector3d p;

    int CLUSTERS_NUM = 4;
    int cluster_id = 0;
    int ROBOTS_NUM = 3;
    int OBSTACLES_NUM = 1;
    int ID = 0;
    double ROBOT_FOV = 120.0;
    double ROBOT_RANGE = 8.0;
    double SAFETY_DIST = 2.0;

    bool got_target;


    

public:


    myNode(): nh_("~"), controller(2.09, 2.0, 8.0, 11, 2, 2)
    {
        std::cout << "Constructor called" << std::endl;
        nh_.getParam("ID", ID);
        nh_.getParam("ROBOTS_NUM", ROBOTS_NUM);
        nh_.getParam("CLUSTERS_NUM", CLUSTERS_NUM);
        nh_.getParam("OBSTACLES_NUM", OBSTACLES_NUM);
        sub = n.subscribe<geometry_msgs::Twist> ("/cmd_vel_in", 1, &myNode::vel_callback, this);
        neighbors_sub = n.subscribe<geometry_msgs::PoseArray>("/neighbors_topic", 1, &myNode::neigh_callback, this);
        pose_sub = n.subscribe<nav_msgs::Odometry>("odom", 1, &myNode::odom_callback, this);
        target_sub = n.subscribe<nav_msgs::Odometry>("/target", 1, &myNode::target_callback, this);
        obstacles_sub = n.subscribe<geometry_msgs::PoseArray>("/obstacles", 1, &myNode::obs_callback, this);
        pub = n.advertise<geometry_msgs::TwistStamped> ("/cmd_vel_out", 1);
        communication_pub = n.advertise<geometry_msgs::PoseArray> ("/detections", 1);

        timer = n.createTimer(ros::Duration(0.1), &myNode::timerCallback,this);

        auto dv = std::div(ID, ROBOTS_NUM);
        cluster_id = dv.quot;
        std::cout << "Cluster number " << cluster_id << std::endl;

        
        // Neighbors
        p_js_i.resize(2, CLUSTERS_NUM*ROBOTS_NUM-1);
        p_js_i.setZero();

        // mates.resize(ROBOTS_NUM-1, Eigen::Vector2d::Zero());
        // enemies.resize(ROBOTS_NUM*(CLUSTERS_NUM-1), Eigen::Vector2d::Zero());


        // Obstacles (static)
        obs_i.resize(2, OBSTACLES_NUM);
        obs_i.setOnes();
        obs_i = obs_i * 100.0;

        obs.resize(2, OBSTACLES_NUM);
        obs.setOnes();
        obs = obs * 100.0;

        got_target = false;

        controller.setGamma(1.0, 5.0, 0.1);
        controller.setVelBounds(-1.0, 1.0);
        controller.setVerbose(false);

        std::cout << "Constructor finished" << std::endl;
        std::cout << "Hi! I'm robot number " << ID << std::endl;


    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        p(0) = msg->pose.pose.position.x;
        p(1) = msg->pose.pose.position.y;
        p(2) = tf2::getYaw(msg->pose.pose.orientation);
    }

    void neigh_callback(const geometry_msgs::PoseArray::ConstPtr &msg)
    {
        // std::cout << "Entering neighbors callback" << std::endl;
        for (int i = 0; i < CLUSTERS_NUM*ROBOTS_NUM; i++)
        {
            int c = i;
            if (c > ID) {c = i - 1;}
            if (i != ID)
            {
                p_js_i(0, c) = msg->poses[i].position.x;
                p_js_i(1, c) = msg->poses[i].position.y;
            }
        }

        // std::cout << "Neighbors: " << p_js_i << std::endl;
    }


    void obs_callback(const geometry_msgs::PoseArray::ConstPtr &msg)
    {
        for (int i = 0; i < OBSTACLES_NUM; i++)
        {
            obs(0, i) = msg->poses[i].position.x;
            obs(1, i) = msg->poses[i].position.y;
        }
    }

    void target_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        p_target(0) = msg->pose.pose.position.x;
        p_target(1) = msg->pose.pose.position.y;
        got_target = true;
    }


    void timerCallback(const ros::TimerEvent&)
    {
        if (!got_target)
        {
            return;
        }
        Eigen::Matrix2d R_w_i;
        R_w_i << cos(p(2)), sin(p(2)),
                -sin(p(2)), cos(p(2));

        // Split teammates and enemies
        std::vector<Eigen::Vector2d> mates, enemies;
        for (int i = 0; i < CLUSTERS_NUM*ROBOTS_NUM; i++)
        {
            int c = i;
            if (c > ID) {c = i - 1;}
            // Teammates
            if (i >= ROBOTS_NUM*cluster_id && i < ROBOTS_NUM*(cluster_id+1))
            {
                // std::cout << "Robot " << i << " is a teammate" << std::endl;
                if (i != ID)
                {
                    mates.push_back(p_js_i.col(c));
                }
            } else
            {
                // std::cout << "Robot " << i << " is an enemy" << std::endl;
                enemies.push_back(p_js_i.col(c));
            }
        }

        // std::cout << "NUmber of mates: " << mates.size() << std::endl;
        // std::cout << "NUmber of enemies: " << enemies.size() << std::endl;

        // Communicate global position of enemies
        geometry_msgs::PoseArray comm_msg;
        comm_msg.header.stamp = ros::Time::now();
        comm_msg.header.frame_id = "odom";
        for (int i = 0; i < enemies.size(); i++)
        {
            geometry_msgs::Pose pose;
            Eigen::Vector2d e = enemies[i];
            Eigen::Vector2d e_glob = p.head(2) + R_w_i.inverse() * e;
            pose.position.x = e_glob(0);
            pose.position.y = e_glob(1);
            // std::cout << "Enemy " << i << " : " << pose.position.x << " " << pose.position.y << std::endl;
            comm_msg.poses.push_back(pose);
        }
        communication_pub.publish(comm_msg);

        // Convert target from global to local
        Eigen::Vector2d p_t_i;
        double dx = p_target(0) - p(0);
        double dy = p_target(1) - p(1);
        p_t_i(0) = dx * cos(p(2)) + dy * sin(p(2));
        p_t_i(1) = -dx * sin(p(2)) + dy * cos(p(2));
        std::cout << "Local target position: " << p_t_i.transpose() << std::endl;

        // Convert obstacles from global to local
        for (int i = 0; i < OBSTACLES_NUM; i++)
        {
            double dx_obs = obs(0, i) - p(0);
            double dy_obs = obs(1, i) - p(1);
            obs_i(0, i) = dx_obs * cos(p(2)) + dy_obs * sin(p(2));
            obs_i(1, i) = -dx_obs * sin(p(2)) + dy_obs * cos(p(2));
            std::cout << "local obs " << i << " : " << obs_i.col(i).transpose() << std::endl;
        }
            

        Eigen::Vector3d uopt;

        // Eigen::Vector2d config_centroid = p_js_i.rowwise().sum() / (ROBOTS_NUM);
        Eigen::Vector2d config_centroid;
        config_centroid.setZero();
        for (int i = 0; i < mates.size(); i++)
        {
            config_centroid += mates[i];
        }
        config_centroid = config_centroid / ROBOTS_NUM;
        std::cout << "Centroid: " << config_centroid.transpose() << std::endl;

        Eigen::Vector3d u_star, uopt_global;
        // u_star.head(2) = p_t_i - Eigen::Vector2d(0.5*ROBOT_RANGE, 0.0);
        u_star.head(2) = p_t_i - config_centroid;
        u_star(2) = atan2(p_t_i(1), p_t_i(0));
        u_star = 0.8 * u_star;
        std::cout << "Desired input : " << u_star.transpose() << std::endl;

        geometry_msgs::TwistStamped vel_msg;
        // std::cout << "obs-i : " << obs_i << std::endl;
        if (!controller.applyCbf(uopt, u_star, p_js_i, p_t_i, obs_i, mates))
        {
            std::cout << "Local optimal vel: " << uopt.transpose() << std::endl;
            uopt_global.head(2) = R_w_i.transpose() * uopt.head(2);
            uopt_global(2) = uopt(2);
            std::cout << "Global optimal vel: " << uopt_global.transpose() << std::endl;
            vel_msg.twist.linear.x = uopt_global(0);
            vel_msg.twist.linear.y = uopt_global(1);
            vel_msg.twist.angular.z = uopt_global(2); 
            pub.publish(vel_msg);
            ROS_INFO("CBF SUCCESS");
        }
        else
        {
            ROS_INFO("CBF FAILED");
            ROS_ERROR("CBF FAILED");
        }
    }


    void vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        Eigen::Vector3d u_global;
        u_global(0) = msg->linear.x;
        u_global(1) = msg->linear.y;
        u_global(2) = msg->angular.z;

        Eigen::MatrixXd R_w_i;                          // rotation matrix from global to local
        R_w_i.resize(3,3);
        R_w_i << cos(p_i(2)), -sin(p_i(2)), 0,
                sin(p_i(2)), cos(p_i(2)), 0,
                0, 0, 1;
        ustar = R_w_i * u_global;

        // ustar(0) = msg->linear.x;
        // ustar(1) = msg->linear.y;
        // ustar(2) = msg->angular.z;
    }

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    ros::init(argc, argv, "front_end");
    myNode node;

    ros::spin();

    return 0;
}