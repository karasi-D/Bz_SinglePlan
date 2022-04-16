#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <bz_plan/dynamicParamConfig.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "trajectory_generator.h"
#include "bezier_base.h"
#include "data_type.h"
#include "fmm_utils.h"
#include "a_star.h"
#include "backward.hpp"

#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"

using namespace std;
using namespace Eigen;
using namespace sdf_tools;

namespace backward {
backward::SignalHandling sh;
}

// simulation param from launch file
double _vis_traj_width;
double _resolution, _inv_resolution;
double _cloud_margin, _cube_margin, _check_horizon, _stop_horizon;
double _x_size, _y_size, _z_size, _x_local_size, _y_local_size, _z_local_size;    
double _MAX_Vel, _MAX_Acc;
bool   _is_use_fm, _is_proj_cube, _is_limit_vel, _is_limit_acc;
int    _step_length, _max_inflate_iter, _traj_order;
double _minimize_order;

// useful global variables
nav_msgs::Odometry _odom;
bool _has_odom  = false;
bool _has_map   = false;
bool _has_target= false;
bool _has_traj  = false;
bool _is_emerg  = false;
bool _is_init   = true;

Vector3d _start_pt, _start_vel, _start_acc, _end_pt;
double _init_x, _init_y, _init_z, group_distance;
Vector3d _map_origin;
double _pt_max_x, _pt_min_x, _pt_max_y, _pt_min_y, _pt_max_z, _pt_min_z;
int _max_x_id, _max_y_id, _max_z_id, _max_local_x_id, _max_local_y_id, _max_local_z_id;
int _traj_id = 1;
COLLISION_CELL _free_cell(0.0);
COLLISION_CELL _obst_cell(1.0);
// ros related
ros::Subscriber _map_sub, _pts_sub, _odom_sub;
ros::Publisher _fm_path_vis_pub, _local_map_vis_pub, _inf_map_vis_pub, _corridor_vis_pub, _traj_vis_pub, _grid_path_vis_pub, _nodes_vis_pub, _traj_pub, _checkTraj_vis_pub, _stopTraj_vis_pub;

// trajectory related
int _seg_num;
VectorXd _seg_time;
MatrixXd _bezier_coeff;

// bezier basis constant
MatrixXd _MQM, _FM;
VectorXd _C, _Cv, _Ca, _Cj;

// useful object
quadrotor_msgs::PolynomialTrajectory _traj;
ros::Time _start_time = ros::TIME_MAX;
TrajectoryGenerator _trajectoryGenerator;
CollisionMapGrid * collision_map       = new CollisionMapGrid();
CollisionMapGrid * collision_map_local = new CollisionMapGrid();
CorridorGenerate * corridor_manager    = new CorridorGenerate();
gridPathFinder * path_finder           = new gridPathFinder();

void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
void rcvOdometryCallbck(const nav_msgs::Odometry odom);

void trajPlanning();
bool checkExecTraj();
bool checkCoordObs(Vector3d checkPt);
vector<pcl::PointXYZ> pointInflate( pcl::PointXYZ pt);
void sortPath(vector<Vector3d> & path_coord, vector<double> & time);
void timeAllocation(vector<Cube> & corridor, vector<double> time);
void timeAllocation(vector<Cube> & corridor);

// ego_planner全局路径显示 topic:/odom_visualization/path
void visPath(vector<Vector3d> path);
void visCorridor(vector<Cube> corridor);
void visGridPath( vector<Vector3d> grid_path);
void visExpNode( vector<GridNodePtr> nodes);
void visBezierTrajectory(MatrixXd polyCoeff, VectorXd time);

//define dynamic call back function 
void paramCallback(bz_plan::dynamicParamConfig& config) 
{ 
    ROS_INFO("Request: %s", config.is_use_fm?"is_use_fm = True":"is_use_fm = False");
    _is_use_fm = config.is_use_fm;
}

void rcvOdometryCallbck(const nav_msgs::Odometry odom)
{
    // if (odom.header.frame_id != "uav") 
    //     return ;
    
    _odom = odom;
    _has_odom = true;
    
    _start_pt(0)  = _odom.pose.pose.position.x;
    _start_pt(1)  = _odom.pose.pose.position.y;
    _start_pt(2)  = _odom.pose.pose.position.z;    

    _start_vel(0) = _odom.twist.twist.linear.x;
    _start_vel(1) = _odom.twist.twist.linear.y;
    _start_vel(2) = _odom.twist.twist.linear.z;    

    _start_acc(0) = _odom.twist.twist.angular.x;
    _start_acc(1) = _odom.twist.twist.angular.y;
    _start_acc(2) = _odom.twist.twist.angular.z;    

    // if not a number return true
    if( std::isnan(_odom.pose.pose.position.x) || std::isnan(_odom.pose.pose.position.y) || std::isnan(_odom.pose.pose.position.z))
        return;
    
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(_odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z) );
    transform.setRotation(tf::Quaternion(0, 0, 0, 1.0));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "quadrotor"));
}

// get target point  --> trajPlanning();
void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    if(wp.poses[0].pose.position.z < 0.0)
        return;

    _is_init = false;
    _end_pt << wp.poses[0].pose.position.x,
               wp.poses[0].pose.position.y,
               wp.poses[0].pose.position.z;

    _has_target = true;
    _is_emerg   = true;
    if(_is_use_fm)
        ROS_INFO("[Fast Marching Node] receive the way-points: %f, %f, %f", _end_pt(0), _end_pt(1), _end_pt(2));
    else
        ROS_INFO("[A star Node] receive the way-points: %f, %f, %f", _end_pt(0), _end_pt(1), _end_pt(2));

    trajPlanning(); 
}

Vector3d _local_origin;
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(pointcloud_map, cloud);
    
    if((int)cloud.points.size() == 0)
        return;

    delete collision_map_local;

    //ros::Time time_1 = ros::Time::now();
    collision_map->RestMap();
    
    // (int)((_start_pt(0) - _x_local_size/2.0)  * _inv_resolution + 0.5) 向上取整操作
    double local_c_x = (int)((_start_pt(0) - _x_local_size/2.0)  * _inv_resolution + 0.5) * _resolution;
    double local_c_y = (int)((_start_pt(1) - _y_local_size/2.0)  * _inv_resolution + 0.5) * _resolution;
    double local_c_z = (int)((_start_pt(2) - _z_local_size/2.0)  * _inv_resolution + 0.5) * _resolution;

    _local_origin << local_c_x, local_c_y, local_c_z;

    Translation3d origin_local_translation( _local_origin(0), _local_origin(1), _local_origin(2));
    Quaterniond origin_local_rotation(1.0, 0.0, 0.0, 0.0);

    Affine3d origin_local_transform = origin_local_translation * origin_local_rotation;
    
    double _buffer_size = 2 * _MAX_Vel;
    double _x_buffer_size = _x_local_size + _buffer_size;
    double _y_buffer_size = _y_local_size + _buffer_size;
    double _z_buffer_size = _z_local_size + _buffer_size;

    collision_map_local = new CollisionMapGrid(origin_local_transform, "world", _resolution, _x_buffer_size, _y_buffer_size, _z_buffer_size, _free_cell);

    vector<pcl::PointXYZ> inflatePts(20);
    pcl::PointCloud<pcl::PointXYZ> cloud_inflation;
    pcl::PointCloud<pcl::PointXYZ> cloud_local;

    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {   
        auto mk = cloud.points[idx];
        pcl::PointXYZ pt(mk.x, mk.y, mk.z);
        Vector3d mkPt(mk.x, mk.y, mk.z);

        // if( fabs(pt.x - _start_pt(0)) > _x_local_size / 2.0 || fabs(pt.y - _start_pt(1)) > _y_local_size / 2.0 || fabs(pt.z - _start_pt(2)) > _z_local_size / 2.0 )
        //     continue; 
        
        cloud_local.push_back(pt);
        inflatePts = pointInflate(pt);
        for(int i = 0; i < (int)inflatePts.size(); i++)
        {   
            pcl::PointXYZ inf_pt = inflatePts[i];
            Vector3d addPt(inf_pt.x, inf_pt.y, inf_pt.z);
            collision_map_local->Set3d(addPt, _obst_cell);
            collision_map->Set3d(addPt, _obst_cell);
            if((mkPt - addPt).norm() < 0.0001) continue;
            cloud_inflation.push_back(inf_pt);
        }
    }

    _has_map = true;

    cloud_inflation.width = cloud_inflation.points.size();
    cloud_inflation.height = 1;
    cloud_inflation.is_dense = true;
    cloud_inflation.header.frame_id = "world";

    cloud_local.width = cloud_local.points.size();
    cloud_local.height = 1;
    cloud_local.is_dense = true;
    cloud_local.header.frame_id = "world";

    sensor_msgs::PointCloud2 inflateMap, localMap;
    
    pcl::toROSMsg(cloud_inflation, inflateMap);
    pcl::toROSMsg(cloud_local, localMap);
    _inf_map_vis_pub.publish(inflateMap);
    _local_map_vis_pub.publish(localMap);
    // printf("[plan node] cloud_inflation size: %d; cloud_local size: %d\n", (int)cloud_inflation.size(), (int)cloud_local.size());

    //ros::Time time_3 = ros::Time::now();
    //ROS_WARN("Time in receving the map is %f", (time_3 - time_1).toSec());

    if( checkExecTraj() == true )
        trajPlanning(); 
}

vector<pcl::PointXYZ> pointInflate( pcl::PointXYZ pt)
{
    // 与实际点云障碍物间保持安全距离 _cloud_margin
    int num   = int(_cloud_margin * _inv_resolution);
    int num_z = max(1, num / 2);
    vector<pcl::PointXYZ> infPts(20);
    pcl::PointXYZ pt_inf;

    for(int x = -num ; x <= num; x ++ )
        for(int y = -num ; y <= num; y ++ )
            for(int z = -num_z ; z <= num_z; z ++ )
            {
                pt_inf.x = pt.x + x * _resolution;
                pt_inf.y = pt.y + y * _resolution;
                pt_inf.z = pt.z + z * _resolution;

                infPts.push_back( pt_inf );
            }

    return infPts;
}

bool checkExecTraj()
{   
    if( _has_traj == false ) 
        return false;

    Vector3d traj_pt;

    visualization_msgs::Marker _check_traj_vis, _stop_traj_vis;

    geometry_msgs::Point pt;
    _stop_traj_vis.header.stamp    = _check_traj_vis.header.stamp    = ros::Time::now();
    _stop_traj_vis.header.frame_id = _check_traj_vis.header.frame_id = "world";
    
    _check_traj_vis.ns = "trajectory/check_trajectory";
    _stop_traj_vis.ns  = "trajectory/stop_trajectory";

    _stop_traj_vis.id     = _check_traj_vis.id = 0;
    _stop_traj_vis.type   = _check_traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _stop_traj_vis.action = _check_traj_vis.action = visualization_msgs::Marker::ADD;

    _stop_traj_vis.scale.x = 2.0 * _vis_traj_width;
    _stop_traj_vis.scale.y = 2.0 * _vis_traj_width;
    _stop_traj_vis.scale.z = 2.0 * _vis_traj_width;

    _check_traj_vis.scale.x = 1.5 * _vis_traj_width;
    _check_traj_vis.scale.y = 1.5 * _vis_traj_width;
    _check_traj_vis.scale.z = 1.5 * _vis_traj_width;

    _check_traj_vis.pose.orientation.x = 0.0;
    _check_traj_vis.pose.orientation.y = 0.0;
    _check_traj_vis.pose.orientation.z = 0.0;
    _check_traj_vis.pose.orientation.w = 1.0;

    _stop_traj_vis.pose = _check_traj_vis.pose;

    _stop_traj_vis.color.r = 0.0;
    _stop_traj_vis.color.g = 1.0;
    _stop_traj_vis.color.b = 0.0;
    _stop_traj_vis.color.a = 1.0;

    _check_traj_vis.color.r = 0.0;
    _check_traj_vis.color.g = 0.0;
    _check_traj_vis.color.b = 1.0;
    _check_traj_vis.color.a = 1.0;

    double t_s = max(0.0, (_odom.header.stamp - _start_time).toSec());      
    int idx;
    for (idx = 0; idx < _seg_num; ++idx)
    {
        if( t_s  > _seg_time(idx) && idx + 1 < _seg_num)
            t_s -= _seg_time(idx);
        else 
            break;
    }

    double duration = 0.0;
    double t_ss;
    for(int i = idx; i < _seg_num; i++ )
    {
        t_ss = (i == idx) ? t_s : 0.0;
        for(double t = t_ss; t < _seg_time(i); t += 0.01)
        {
            double t_d = duration + t - t_ss;
            if( t_d > _check_horizon ) break;
            traj_pt = _trajectoryGenerator.getPosFromBezier( _bezier_coeff, _C, _traj_order, t/_seg_time(i), i );
            pt.x = traj_pt(0) = _seg_time(i) * traj_pt(0); 
            pt.y = traj_pt(1) = _seg_time(i) * traj_pt(1);
            pt.z = traj_pt(2) = _seg_time(i) * traj_pt(2);

            _check_traj_vis.points.push_back(pt);

            if( t_d <= _stop_horizon ) 
                _stop_traj_vis.points.push_back(pt);

            if( checkCoordObs(traj_pt))
            {   
                ROS_WARN("predicted collision time is %f ahead", t_d);
                
                if( t_d <= _stop_horizon ) 
                {   
                    ROS_ERROR("emergency occurs in time is %f ahead", t_d);
                    _is_emerg = true;
                }

                _checkTraj_vis_pub.publish(_check_traj_vis);
                _stopTraj_vis_pub.publish(_stop_traj_vis); 

                return true;
            }
        }
        duration += _seg_time(i) - t_ss;
    }

    _checkTraj_vis_pub.publish(_check_traj_vis); 
    _stopTraj_vis_pub.publish(_stop_traj_vis); 

    return false;
}

bool checkCoordObs(Vector3d checkPt)
{       
    if(collision_map->Get(checkPt(0), checkPt(1), checkPt(2)).first.occupancy > 0.0 )
        return true;

    return false;
}

// 速度场 计算当前点的通行速度
double velMapping(double d, double max_v)
{   
    double vel;

    if( d <= 0.25)
        vel = 2.0 * d * d;
    else if(d > 0.25 && d <= 0.75)
        vel = 1.5 * d - 0.25;
    else if(d > 0.75 && d <= 1.0)
        vel = - 2.0 * (d - 1.0) * (d - 1.0) + 1;  
    else
        vel = 1.0;

    return vel * max_v;
}

void trajPlanning()
{   
        // ROS_WARN("state: _has_target= %d, _has_map =%d, _has_odom =%d", _has_target,_has_map,_has_odom);
    if( _has_target == false || _has_map == false || _has_odom == false) 
        return;
    
    vector<Cube> corridor;
    if(_is_use_fm)
    {
        // use fmm find path
        ros::Time time_1 = ros::Time::now();
        float oob_value = INFINITY;
        auto EDT = collision_map_local->ExtractDistanceField(oob_value);
        ros::Time time_2 = ros::Time::now();
        ROS_WARN("time in generate EDT is %f", (time_2 - time_1).toSec());

        unsigned int idx;
        double max_vel = _MAX_Vel * 0.75; 
        vector<unsigned int> obs;            
        Vector3d pt;
        vector<int64_t> pt_idx;
        double flow_vel;

        unsigned int size_x = (unsigned int)(_max_x_id);
        unsigned int size_y = (unsigned int)(_max_y_id);
        unsigned int size_z = (unsigned int)(_max_z_id);

        Coord3D dimsize {size_x, size_y, size_z};
        FMGrid3D grid_fmm(dimsize);

        // 计算速度场 grid_fmm
        for(unsigned int k = 0; k < size_z; k++)
        {
            for(unsigned int j = 0; j < size_y; j++)
            {
                for(unsigned int i = 0; i < size_x; i++)
                {
                    idx = k * size_y * size_x + j * size_x + i;
                    pt << (i + 0.5) * _resolution + _map_origin(0), 
                          (j + 0.5) * _resolution + _map_origin(1), 
                          (k + 0.5) * _resolution + _map_origin(2);

                    Vector3i index = collision_map_local->LocationToGridIndex(pt);

                    if(collision_map_local->Inside(index))
                    {
                        double d = sqrt(EDT.GetImmutable(index).first.distance_square) * _resolution;
                        flow_vel = velMapping(d, max_vel);
                    }
                    else
                        flow_vel = max_vel;
    
                    if( k == 0 || k == (size_z - 1) || j == 0 || j == (size_y - 1) || i == 0 || i == (size_x - 1) )
                        flow_vel = 0.0;

                    grid_fmm[idx].setOccupancy(flow_vel);
                    if (grid_fmm[idx].isOccupied())
                        obs.push_back(idx);
                }
            }
        }
        
        grid_fmm.setOccupiedCells(std::move(obs));
        grid_fmm.setLeafSize(_resolution);

        Vector3d startIdx3d = (_start_pt - _map_origin) * _inv_resolution; 
        Vector3d endIdx3d   = (_end_pt   - _map_origin) * _inv_resolution;

        Coord3D goal_point = {(unsigned int)startIdx3d[0], (unsigned int)startIdx3d[1], (unsigned int)startIdx3d[2]};
        Coord3D init_point = {(unsigned int)endIdx3d[0],   (unsigned int)endIdx3d[1],   (unsigned int)endIdx3d[2]}; 

        unsigned int startIdx;
        vector<unsigned int> startIndices;
        grid_fmm.coord2idx(init_point, startIdx);
        
        startIndices.push_back(startIdx);
        
        unsigned int goalIdx;
        grid_fmm.coord2idx(goal_point, goalIdx);
        grid_fmm[goalIdx].setOccupancy(max_vel);     

        Solver<FMGrid3D>* fm_solver = new FMMStar<FMGrid3D>("FMM*_Dist", TIME); // LSM, FMM
    
        fm_solver->setEnvironment(&grid_fmm);
        fm_solver->setInitialAndGoalPoints(startIndices, goalIdx);

        ros::Time time_bef_fm = ros::Time::now();
        if(fm_solver->compute(max_vel) == -1)
        {
            ROS_WARN("[Fast Marching Node] No path can be found");
            _traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
            _traj_pub.publish(_traj);
            _has_traj = false;

            return;
        }
        ros::Time time_aft_fm = ros::Time::now();
        ROS_WARN("[FMM] Time in Fast Marching computing is %f", (time_aft_fm - time_bef_fm).toSec() );

        Path3D path3D;
        vector<double> path_vels, time;
        GradientDescent< FMGrid3D > grad3D;
        grid_fmm.coord2idx(goal_point, goalIdx);

        // find a path by gradient descent
        /// input: grid_fmm, goalIdx:从goal开始findPath
        /// output: path3D, path_vels, time
        /// return: success 1, false -1
        if(grad3D.gradient_descent(grid_fmm, goalIdx, path3D, path_vels, time) == -1)
        {
            ROS_WARN("[Fast Marching Node] FMM failed, valid path not exists");
            if(_has_traj && _is_emerg)
            {
                _traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
                _traj_pub.publish(_traj);
                _has_traj = false;
            } 
            return;
        }

        // 将 path3D 按照可视化地图规整化为 path_coord, 并将其可视化
        vector<Vector3d> path_coord;
        path_coord.push_back(_start_pt);                
        double coord_x, coord_y, coord_z;
        for( int i = 0; i < (int)path3D.size(); i++)
        {
            coord_x = max(min( (path3D[i][0]+0.5) * _resolution + _map_origin(0), _x_size), -_x_size);
            coord_y = max(min( (path3D[i][1]+0.5) * _resolution + _map_origin(1), _y_size), -_y_size);
            coord_z = max(min( (path3D[i][2]+0.5) * _resolution, _z_size), 0.0);

            Vector3d pt(coord_x, coord_y, coord_z);
            path_coord.push_back(pt);
        }
        visPath(path_coord);

        // 简化路径(sortPath) + 生成安全区-飞行走廊，即生成cube并膨胀处理
        ros::Time time_bef_corridor = ros::Time::now();    
        sortPath(path_coord, time);
        corridor = corridor_manager->corridorGeneration(path_coord, time, collision_map);
        ros::Time time_aft_corridor = ros::Time::now();
        ROS_WARN("[FMM] Time consume in corridor generation is %f", (time_aft_corridor - time_bef_corridor).toSec());

        timeAllocation(corridor, time);
        visCorridor(corridor);

        delete fm_solver;
    }
    else
    {   // use astar find path
        path_finder->linkLocalMap(collision_map_local, _local_origin);
        path_finder->AstarSearch(_start_pt, _end_pt);
        vector<Vector3d> gridPath = path_finder->getPath();
        vector<GridNodePtr> searchedNodes = path_finder->getVisitedNodes();
        path_finder->resetLocalMap();
        
        visGridPath(gridPath);
        visExpNode(searchedNodes);

        ros::Time time_bef_corridor = ros::Time::now();    
        corridor = corridor_manager->corridorGeneration(gridPath, collision_map);
        ros::Time time_aft_corridor = ros::Time::now();
        ROS_WARN("[Astar] Time consume in corridor generation is %f", (time_aft_corridor - time_bef_corridor).toSec());

        timeAllocation(corridor);
        visCorridor(corridor);
    }

    MatrixXd pos = MatrixXd::Zero(2,3);
    MatrixXd vel = MatrixXd::Zero(2,3);
    MatrixXd acc = MatrixXd::Zero(2,3);

    pos.row(0) = _start_pt;
    pos.row(1) = _end_pt;    
    vel.row(0) = _start_vel;
    acc.row(0) = _start_acc;

    // 后端：轨迹优化，基于BezierPolyCoeff    
    double obj;
    ros::Time time_bef_opt = ros::Time::now();

    if(_trajectoryGenerator.BezierPloyCoeffGeneration
        ( corridor, _MQM, pos, vel, acc, _MAX_Vel, _MAX_Acc, _traj_order, _minimize_order, 
         _cube_margin, _is_limit_vel, _is_limit_acc, obj, _bezier_coeff ) == -1 )
    {
        ROS_WARN("Cannot find a feasible and optimal solution, somthing wrong with the mosek solver");
          
        if(_has_traj && _is_emerg)
        {
            _traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
            _traj_pub.publish(_traj);
            _has_traj = false;
        } 
    }
    else
    {   
        _seg_num = corridor.size();
        _seg_time.resize(_seg_num);

        for(int i = 0; i < _seg_num; i++)
            _seg_time(i) = corridor[i].t;

        _is_emerg = false;
        _has_traj = true;
        _start_time = _odom.header.stamp;
        _traj = _trajectoryGenerator.getBezierTraj(_bezier_coeff, _seg_time, _traj_order,  _seg_num,  _traj_id, _start_time );
        
        _traj_pub.publish(_traj);
        _traj_id ++;
        visBezierTrajectory(_bezier_coeff, _seg_time);
    }

    ros::Time time_aft_opt = ros::Time::now();

    ROS_WARN("The objective of the program is %f", obj);
    ROS_WARN("The time consumation of the program is %f", (time_aft_opt - time_bef_opt).toSec());
}

void sortPath(vector<Vector3d> & path_coord, vector<double> & time)
{   
    vector<Vector3d> path_tmp;
    vector<double> time_tmp;

    for (int i = 0; i < (int)path_coord.size(); i += 1)
    {
        if( i )
            if( std::isinf(time[i]) || time[i] == 0.0 || time[i] == time[i-1] )
                continue;

        if( (path_coord[i] - _end_pt).norm() < 0.2)
            break;

        path_tmp.push_back(path_coord[i]);
        time_tmp.push_back(time[i]);
    }
    path_coord = path_tmp;
    time       = time_tmp;
}   

void timeAllocation(vector<Cube> & corridor, vector<double> time)
{   
    vector<double> tmp_time;

    for(int i  = 0; i < (int)corridor.size() - 1; i++)
    {   
        double duration  = (corridor[i].t - corridor[i+1].t);
        tmp_time.push_back(duration);
    }    
    double lst_time = corridor.back().t;
    tmp_time.push_back(lst_time);

    vector<Vector3d> points;
    points.push_back (_start_pt);
    for(int i = 1; i < (int)corridor.size(); i++)
        points.push_back(corridor[i].center);

    points.push_back (_end_pt);

    double _Vel = _MAX_Vel * 0.6;
    double _Acc = _MAX_Acc * 0.6;

    Eigen::Vector3d initv = _start_vel;
    for(int i = 0; i < (int)points.size() - 1; i++)
    {
        double dtxyz;

        Eigen::Vector3d p0   = points[i];    
        Eigen::Vector3d p1   = points[i + 1];
        Eigen::Vector3d d    = p1 - p0;            
        Eigen::Vector3d v0(0.0, 0.0, 0.0);        
        
        if( i == 0) v0 = initv;

        // std: d.norm()= sqrt(d^2)；eigen: 点乘A.dot(B) = A·B，叉乘A.cross(B) = A * B
        double D    = d.norm();                   
        double V0   = v0.dot(d / D);              
        double aV0  = fabs(V0);                   

        double acct = (_Vel - V0) / _Acc * ((_Vel > V0)?1:-1); 
        double accd = V0 * acct + (_Acc * acct * acct / 2) * ((_Vel > V0)?1:-1);
        double dcct = _Vel / _Acc;                                              
        double dccd = _Acc * dcct * dcct / 2;                                   
        // 讨论两段间距 与 加速减速的对应情况，全程以最大加速度飞行
        if (D < aV0 * aV0 / (2 * _Acc)) // 从aV0直接减速至零的距离
        {                 
            double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
            double t2 = aV0 / _Acc;
            dtxyz     = t1 + t2;                 
        }
        else if (D < accd + dccd) // 从V0加速至最大，立即减速至0的距离
        {
            double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
            double t2 = (-aV0 + sqrt(aV0 * aV0 + _Acc * D - aV0 * aV0 / 2)) / _Acc;
            double t3 = (aV0 + _Acc * t2) / _Acc;
            dtxyz     = t1 + t2 + t3;    
        }
        else // 从V0加速至最大，匀速飞行一段，然后减速至0
        {
            double t1 = acct;                              
            double t2 = (D - accd - dccd) / _Vel;
            double t3 = dcct;
            dtxyz     = t1 + t2 + t3;
        }

        if(dtxyz < tmp_time[i] * 0.5)
            tmp_time[i] = dtxyz; // if FM given time in this segment is rediculous long, use the new value
    }

    for(int i = 0; i < (int)corridor.size(); i++)
        corridor[i].t = tmp_time[i];
}

void timeAllocation(vector<Cube> & corridor)
{   
    vector<Vector3d> points;
    points.push_back (_start_pt);

    for(int i = 1; i < (int)corridor.size(); i++)
        points.push_back(corridor[i].center);

    points.push_back (_end_pt);

    double _Vel = _MAX_Vel * 0.6;
    double _Acc = _MAX_Acc * 0.6;

    for (int k = 0; k < (int)points.size() - 1; k++)
    {
          double dtxyz;
          Vector3d p0   = points[k];        
          Vector3d p1   = points[k + 1];    
          Vector3d d    = p1 - p0;          
          Vector3d v0(0.0, 0.0, 0.0);       
          
          if( k == 0) v0 = _start_vel;

          double D    = d.norm();                  
          double V0   = v0.dot(d / D);             
          double aV0  = fabs(V0);                  

          double acct = (_Vel - V0) / _Acc * ((_Vel > V0)?1:-1);
          double accd = V0 * acct + (_Acc * acct * acct / 2) * ((_Vel > V0)?1:-1);
          double dcct = _Vel / _Acc;                                              
          double dccd = _Acc * dcct * dcct / 2;                                   

          if (D < aV0 * aV0 / (2 * _Acc))
          {               
            double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
            double t2 = aV0 / _Acc;
            dtxyz     = t1 + t2;                 
          }
          else if (D < accd + dccd)
          {
            double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
            double t2 = (-aV0 + sqrt(aV0 * aV0 + _Acc * D - aV0 * aV0 / 2)) / _Acc;
            double t3 = (aV0 + _Acc * t2) / _Acc;
            dtxyz     = t1 + t2 + t3;    
          }
          else
          {
            double t1 = acct;                              
            double t2 = (D - accd - dccd) / _Vel;
            double t3 = dcct;
            dtxyz     = t1 + t2 + t3;                                                                  
          }
          corridor[k].t = dtxyz;
      }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bz_plan_node");
    ros::NodeHandle nh("~");

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _odom_sub = nh.subscribe( "odometry",  1, rcvOdometryCallbck);
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );

    _inf_map_vis_pub   = nh.advertise<sensor_msgs::PointCloud2>("vis_map_inflate", 1);
    _local_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("vis_map_local", 1);
    _traj_vis_pub      = nh.advertise<visualization_msgs::Marker>("trajectory_vis", 1);    
    _corridor_vis_pub  = nh.advertise<visualization_msgs::MarkerArray>("corridor_vis", 1);
    _fm_path_vis_pub   = nh.advertise<visualization_msgs::MarkerArray>("path_vis", 1);
    _grid_path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("grid_path_vis", 1);
    _nodes_vis_pub     = nh.advertise<visualization_msgs::Marker>("expanded_nodes_vis", 1);
    _checkTraj_vis_pub = nh.advertise<visualization_msgs::Marker>("check_trajectory", 1);
    _stopTraj_vis_pub  = nh.advertise<visualization_msgs::Marker>("stop_trajectory", 1);

    _traj_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 10);

    dynamic_reconfigure::Server<bz_plan::dynamicParamConfig> server; 
    dynamic_reconfigure::Server<bz_plan::dynamicParamConfig>::CallbackType f_paramCallback; 

    f_paramCallback = boost::bind(&paramCallback,_1); 
    server.setCallback(f_paramCallback);

    // 所有参数以launch文件为准，这里只是预设定
    nh.param("map/margin",     _cloud_margin, 0.25);
    nh.param("map/resolution", _resolution, 0.2);
    
    nh.param("map/x_size",       _x_size, 50.0);
    nh.param("map/y_size",       _y_size, 50.0);
    nh.param("map/z_size",       _z_size, 5.0 );
    
    nh.param("map/x_local_size", _x_local_size, -20.0);
    nh.param("map/y_local_size", _y_local_size, -20.0);
    nh.param("map/z_local_size", _z_local_size, 5.0 );

    nh.param("planning/init_x",       _init_x,  0.0);
    nh.param("planning/init_y",       _init_y,  0.0);
    nh.param("planning/init_z",       _init_z,  0.0);

    nh.param("group_distance", group_distance, 5.0);

    nh.param("planning/max_vel",       _MAX_Vel,  1.0);
    nh.param("planning/max_acc",       _MAX_Acc,  1.0);
    nh.param("planning/max_inflate",   _max_inflate_iter, 100);
    nh.param("planning/step_length",   _step_length,     2);
    nh.param("planning/cube_margin",   _cube_margin,   0.2);
    nh.param("planning/check_horizon", _check_horizon,10.0);
    nh.param("planning/stop_horizon",  _stop_horizon,  5.0);
    nh.param("planning/is_limit_vel",  _is_limit_vel,  false);
    nh.param("planning/is_limit_acc",  _is_limit_acc,  false);
    // nh.param("planning/is_use_fm",     _is_use_fm,  true);

    nh.param("optimization/min_order",  _minimize_order, 3.0);
    nh.param("optimization/poly_order", _traj_order,    10);

    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
    nh.param("vis/is_proj_cube",   _is_proj_cube, true);

    Bernstein _bernstein;
    if(_bernstein.setParam(3, 12, _minimize_order) == -1)
        ROS_ERROR(" The trajectory order is set beyond the library's scope, please re-set "); 

    _MQM = _bernstein.getMQM()[_traj_order];
    _FM  = _bernstein.getFM()[_traj_order];
    _C   = _bernstein.getC()[_traj_order];
    _Cv  = _bernstein.getC_v()[_traj_order];
    _Ca  = _bernstein.getC_a()[_traj_order];
    _Cj  = _bernstein.getC_j()[_traj_order];

    _map_origin << -_x_size/2.0, -_y_size/2.0, 0.0; // 地图原点：xy平面的中心点
    _pt_max_x = + _x_size / 2.0;
    _pt_min_x = - _x_size / 2.0;
    _pt_max_y = + _y_size / 2.0;
    _pt_min_y = - _y_size / 2.0; 
    _pt_max_z = + _z_size;
    _pt_min_z = 0.0;

    _inv_resolution = 1.0 / _resolution; // 1.0单位内有_inv_resolution个栅格，每个栅格的边长为 _resolution
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);
    _max_local_x_id = (int)(_x_local_size * _inv_resolution);
    _max_local_y_id = (int)(_y_local_size * _inv_resolution);
    _max_local_z_id = (int)(_z_local_size * _inv_resolution);

    Vector3i GLSIZE(_max_x_id, _max_y_id, _max_z_id);
    Vector3i LOSIZE(_max_local_x_id, _max_local_y_id, _max_local_z_id);
    Vector3d _pt_max(_pt_max_x, _pt_max_y, _pt_max_z);
    Vector3d _pt_min(_pt_min_x, _pt_min_y, _pt_min_z);

    path_finder = new gridPathFinder(GLSIZE, LOSIZE);
    path_finder->initGridNodeMap(_resolution, _map_origin);

    Translation3d origin_translation( _map_origin(0), _map_origin(1), 0.0);
    Quaterniond origin_rotation(1.0, 0.0, 0.0, 0.0);
    Affine3d origin_transform = origin_translation * origin_rotation;
    collision_map = new CollisionMapGrid(origin_transform, "world", _resolution, _x_size, _y_size, _z_size, _free_cell);
    corridor_manager = new CorridorGenerate(_pt_min, _pt_max, GLSIZE, _resolution, _max_inflate_iter, _step_length);
    
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();           
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}

visualization_msgs::MarkerArray path_vis; 
void visPath(vector<Vector3d> path)
{
    for(auto & mk: path_vis.markers) 
        mk.action = visualization_msgs::Marker::DELETE;

    _fm_path_vis_pub.publish(path_vis);
    path_vis.markers.clear();

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.ns = "b_traj/fast_marching_path";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.a = 0.6;
    mk.color.r = 1.0;
    mk.color.g = 1.0;
    mk.color.b = 1.0;

    int idx = 0;
    for(int i = 0; i < int(path.size()); i++)
    {
        mk.id = idx;

        mk.pose.position.x = path[i](0); 
        mk.pose.position.y = path[i](1); 
        mk.pose.position.z = path[i](2);  

        mk.scale.x = _resolution;
        mk.scale.y = _resolution;
        mk.scale.z = _resolution;

        idx ++;
        path_vis.markers.push_back(mk);
    }

    _fm_path_vis_pub.publish(path_vis);
}

visualization_msgs::MarkerArray cube_vis;
void visCorridor(vector<Cube> corridor)
{   
    for(auto & mk: cube_vis.markers) 
        mk.action = visualization_msgs::Marker::DELETE;
    
    _corridor_vis_pub.publish(cube_vis);

    cube_vis.markers.clear();

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.ns = "corridor";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.a = 0.2;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 0.5;

    int idx = 0;
    for(int i = 0; i < int(corridor.size()); i++)
    {   
        mk.id = idx;

        mk.pose.position.x = (corridor[i].vertex(0, 0) + corridor[i].vertex(3, 0) ) / 2.0; 
        mk.pose.position.y = (corridor[i].vertex(0, 1) + corridor[i].vertex(1, 1) ) / 2.0; 

        if(_is_proj_cube)
            mk.pose.position.z = 0.0; 
        else
            mk.pose.position.z = (corridor[i].vertex(0, 2) + corridor[i].vertex(4, 2) ) / 2.0; 

        mk.scale.x = (corridor[i].vertex(0, 0) - corridor[i].vertex(3, 0) );
        mk.scale.y = (corridor[i].vertex(1, 1) - corridor[i].vertex(0, 1) );

        if(_is_proj_cube)
            mk.scale.z = 0.05; 
        else
            mk.scale.z = (corridor[i].vertex(0, 2) - corridor[i].vertex(4, 2) );

        idx ++;
        cube_vis.markers.push_back(mk);
    }

    _corridor_vis_pub.publish(cube_vis);
}

void visBezierTrajectory(MatrixXd polyCoeff, VectorXd time)
{   
    visualization_msgs::Marker traj_vis;

    traj_vis.header.stamp       = ros::Time::now();
    traj_vis.header.frame_id    = "world";

    traj_vis.ns = "trajectory/trajectory";
    traj_vis.id = 0;
    traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    
    traj_vis.action = visualization_msgs::Marker::DELETE;
    _checkTraj_vis_pub.publish(traj_vis);
    _stopTraj_vis_pub.publish(traj_vis);

    traj_vis.action = visualization_msgs::Marker::ADD;
    traj_vis.scale.x = _vis_traj_width;
    traj_vis.scale.y = _vis_traj_width;
    traj_vis.scale.z = _vis_traj_width;
    traj_vis.pose.orientation.x = 0.0;
    traj_vis.pose.orientation.y = 0.0;
    traj_vis.pose.orientation.z = 0.0;
    traj_vis.pose.orientation.w = 1.0;
    traj_vis.color.r = 1.0;
    traj_vis.color.g = 0.0;
    traj_vis.color.b = 0.0;
    traj_vis.color.a = 0.6;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();
    
    traj_vis.points.clear();

    Vector3d state;
    geometry_msgs::Point pt;

    int segment_num  = polyCoeff.rows();
    for(int i = 0; i < segment_num; i++ ){
        for (double t = 0.0; t < 1.0; t += 0.05 / time(i), count += 1){
            state = _trajectoryGenerator.getPosFromBezier( polyCoeff, _C, _traj_order, t, i );
            cur(0) = pt.x = time(i) * state(0);
            cur(1) = pt.y = time(i) * state(1);
            cur(2) = pt.z = time(i) * state(2);
            traj_vis.points.push_back(pt);

            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }
    }

    ROS_INFO("[GENERATOR] The length of the trajectory; %.3lfm.", traj_len);
    _traj_vis_pub.publish(traj_vis);
}

visualization_msgs::MarkerArray grid_vis; 
void visGridPath( vector<Vector3d> grid_path )
{   
    for(auto & mk: grid_vis.markers) 
        mk.action = visualization_msgs::Marker::DELETE;

    _grid_path_vis_pub.publish(grid_vis);
    grid_vis.markers.clear();

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.ns = "b_traj/grid_path";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;

    int idx = 0;
    for(int i = 0; i < int(grid_path.size()); i++)
    {
        mk.id = idx;

        mk.pose.position.x = grid_path[i](0); 
        mk.pose.position.y = grid_path[i](1); 
        mk.pose.position.z = grid_path[i](2);  

        mk.scale.x = _resolution;
        mk.scale.y = _resolution;
        mk.scale.z = _resolution;

        idx ++;
        grid_vis.markers.push_back(mk);
    }

    _grid_path_vis_pub.publish(grid_vis);
}

void visExpNode( vector<GridNodePtr> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "b_traj/visited_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.3;
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i]->coord;
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _nodes_vis_pub.publish(node_vis);
}
