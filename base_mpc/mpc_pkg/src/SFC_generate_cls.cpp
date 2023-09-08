


#include <ros/ros.h>
#include <cmath>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_geometry/geometric_utils.h>


#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>


#include <mpc_pkg/SFC_generate_cls.h>
#include <nav_msgs/OccupancyGrid.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

#define PI 3.1415

SFCGenerateNode::SFCGenerateNode()
:logger("SFC_generate", 20)
{

    scan_multi_sub = nh.subscribe("/abb/laser/scan_multi", 1, &SFCGenerateNode::scan_multiCB, this);
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/SFC_generate_node/point_cloud", 10);
    polygon_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("/SFC_generate_node/free_polygon", 10);
    mpc_constraints_service = nh.advertiseService("/SFC_generate_node/get_constraints", &SFCGenerateNode::GetMpcConstraintsCB, this);

    bool enable_logging;
    ros::param::get("/SFC_generate_enable_logging", enable_logging);
    logger.get_enable(enable_logging);
}


void SFCGenerateNode::scan_multiCB(const sensor_msgs::LaserScan::ConstPtr& data)
{

    // std::stringstream ss0;
    // ss0 << data->header.frame_id;
    // std::string str = ss0.str();

    // auto output_msg = "laser scan frame id is " + str;
    // logger.log(output_msg, 1);

    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*data, cloud);

    obs.clear();
    for (size_t i = 0; i < cloud.points.size(); i++)
    {
        Vecf<2> pt;

        pt(0) = cloud.points[i].x;
        pt(1) = cloud.points[i].y;

        Vecf<2> pt1, pt2, pt3, pt4;
        double radiu = 0.75;
        pt1(0) = pt(0) - radiu;
        pt1(1) = pt(1);

        pt2(0) = pt(0) + radiu;
        pt2(1) = pt(1);

        pt3(0) = pt(0);
        pt3(1) = pt(1) + radiu;

        pt4(0) = pt(0);
        pt4(1) = pt(1) - radiu;

        obs.push_back(pt);
        obs.push_back(pt1);
        obs.push_back(pt2);
        obs.push_back(pt3);
        obs.push_back(pt4);
    }

    point_cloud_pub.publish(cloud);
}


bool SFCGenerateNode::GetMpcConstraintsCB(mpc_pkg::MpcConstraint::Request & req, mpc_pkg::MpcConstraint::Response & res)
{
    logger.log("this is get mpc constraints CB");
    const Vec2f origin(- 4, - 4);
    const Vec2f range(8, 8);

    Vec2f current_pose{req.current_state[0], req.current_state[1]};

    path.clear();

    path.push_back(Vec2f(0.0, 0.0));
    for (size_t i=0; i< req.point_list.size(); i++)
    {
        path.push_back(Vec2f( req.point_list[i].x, req.point_list[i].y));
        //std::cout<< "x=" << path[i+1](0) << "y="<<path[i+1](1) << std::endl;
    }


    // Initialize SeedDecomp2D
    EllipsoidDecomp2D decomp(origin, range);
    decomp.set_obs(obs);
    decomp.set_local_bbox(Vec2f(3, 3));
    decomp.dilate(path, 0);
    auto ellipsoids = decomp.get_ellipsoids();

    jsk_recognition_msgs::PolygonArray polygon_list;
    polygon_list.header.frame_id = "base_link";

    for(const auto& poly: decomp.get_polyhedrons()) {
        const auto vertices = cal_vertices(poly);

        geometry_msgs::PolygonStamped polygon_stamped;
        polygon_stamped.header.frame_id = "base_link";
        for (size_t i = 0; i < vertices.size(); i++)
        {
            geometry_msgs::Point32 p;

            p.x = vertices[i](0);
            p.y = vertices[i](1);
            p.z = 0;
            polygon_stamped.polygon.points.push_back(p);
        }


        polygon_list.polygons.push_back(polygon_stamped);
    }


    polygon_pub.publish(polygon_list);

    res.polygon = polygon_list;
    return true;
}