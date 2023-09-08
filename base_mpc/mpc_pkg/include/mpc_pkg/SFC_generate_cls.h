#ifndef SFC_GENERATE_NODE
#define SFC_GENERATE_NODE

#include <ros/ros.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_geometry/geometric_utils.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "mpc_pkg/logger.h"
#include "mpc_pkg/MpcConstraint.h"
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

class SFCGenerateNode
{
    public:
        SFCGenerateNode();
        void scan_multiCB(const sensor_msgs::LaserScan::ConstPtr& data);
        bool GetMpcConstraintsCB(mpc_pkg::MpcConstraint::Request & req, mpc_pkg::MpcConstraint::Response & res);

    private:
        ros::NodeHandle nh;

        ros::Subscriber scan_multi_sub;

        ros::Publisher polygon_pub;
        ros::Publisher point_cloud_pub;
        ros::ServiceServer mpc_constraints_service;

        vec_Vec2f path;
        vec_Vec2f obs;
        Logger logger;


};

#endif