#ifndef _DATA_TYPE_
#define _DATA_TYPE_

#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <math.h>
#include <sdf_tools/collision_map.hpp>

using namespace std;
using namespace Eigen;

#define inf 1>>30

struct Cube;
struct GridNode;
typedef GridNode* GridNodePtr;

struct Cube
{     
      //Eigen::Vector3d p1, p2, p3, p4, p5, p6, p7, p8;    
      Eigen::MatrixXd vertex; // the 8 vertex of a cube
      Eigen::Vector3d center; // the center of the cube
      bool valid;    // indicates whether this cube should be deleted

      double t; // time allocated to this cube
      std::vector< std::pair<double, double> > box;
/*
           P4------------P3 
           /|           /|              ^
          / |          / |              | z
        P1--|---------P2 |              |
         |  P8--------|--p7             |
         | /          | /               /--------> y
         |/           |/               /  
        P5------------P6              / x
*/                                                                                 

      // create a cube using 8 vertex and the center point
      Cube( Eigen::MatrixXd vertex_, Eigen::Vector3d center_)
      {
            vertex = vertex_;
            center = center_;
            valid = true;
            t = 0.0;
            box.resize(3);
      }

      // create a inscribe cube of a ball using the center point and the radius of the ball
      void setVertex( Eigen::MatrixXd vertex_, double resolution_)
      {     
            vertex = vertex_;
            vertex(0,1) -= resolution_ / 2.0;
            vertex(3,1) -= resolution_ / 2.0;
            vertex(4,1) -= resolution_ / 2.0;
            vertex(7,1) -= resolution_ / 2.0;

            vertex(1,1) += resolution_ / 2.0;
            vertex(2,1) += resolution_ / 2.0;
            vertex(5,1) += resolution_ / 2.0;
            vertex(6,1) += resolution_ / 2.0;

            vertex(0,0) += resolution_ / 2.0;
            vertex(1,0) += resolution_ / 2.0;
            vertex(4,0) += resolution_ / 2.0;
            vertex(5,0) += resolution_ / 2.0;

            vertex(2,0) -= resolution_ / 2.0;
            vertex(3,0) -= resolution_ / 2.0;
            vertex(6,0) -= resolution_ / 2.0;
            vertex(7,0) -= resolution_ / 2.0;

            vertex(0,2) += resolution_ / 2.0;
            vertex(1,2) += resolution_ / 2.0;
            vertex(2,2) += resolution_ / 2.0;
            vertex(3,2) += resolution_ / 2.0;

            vertex(4,2) -= resolution_ / 2.0;
            vertex(5,2) -= resolution_ / 2.0;
            vertex(6,2) -= resolution_ / 2.0;
            vertex(7,2) -= resolution_ / 2.0;
            
            setBox();
      }
      
      void setBox()
      {
            box.clear();
            box.resize(3);
            box[0] = std::make_pair( vertex(3, 0), vertex(0, 0) );
            box[1] = std::make_pair( vertex(0, 1), vertex(1, 1) );
            box[2] = std::make_pair( vertex(4, 2), vertex(1, 2) );
      }

      void printBox()
      {
            std::cout<<"center of the cube: \n"<<center<<std::endl;
            std::cout<<"vertex of the cube: \n"<<vertex<<std::endl;
      }

      Cube()
      {  
         center = Eigen::VectorXd::Zero(3);
         vertex = Eigen::MatrixXd::Zero(8, 3);

         valid = true;
         t = 0.0;
         box.resize(3);
      }

      ~Cube(){}
};

struct GridNode
{     
   int id;        // 1--> open set, -1 --> closed set
   Eigen::Vector3d coord;
   Eigen::Vector3i index;
   
   double gScore, fScore;
   GridNodePtr cameFrom;
   std::multimap<double, GridNodePtr>::iterator nodeMapIt;
   double occupancy; 

   std::vector<GridNodePtr> hisNodeList; // use a list to record nodes in its history

   GridNode(Eigen::Vector3i _index)
   {  
      id = 0;
      index = _index;
      
      gScore = inf;
      fScore = inf;
      cameFrom = NULL;
   }

   GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord)
   {  
      id = 0;
      index = _index;
      coord = _coord;

      gScore = inf;
      fScore = inf;
      cameFrom = NULL;
   }

   GridNode(){};
   
   ~GridNode(){};
};

class CorridorGenerate{
private:
      double _pt_max_x, _pt_min_x, _pt_max_y, _pt_min_y, _pt_max_z, _pt_min_z;
      int _max_x_id, _max_y_id, _max_z_id;
      double _resolution;
      int _max_inflate_iter, _step_length;

public:
      CorridorGenerate(){}
      ~CorridorGenerate(){}
      CorridorGenerate(Vector3d & pt_min, Vector3d & pt_max, Vector3i & id_max, double resolution, int max_inflate_iter, int step_length)
      : _resolution(resolution), _max_inflate_iter(max_inflate_iter), _step_length(step_length)
      {
           _pt_min_x = pt_min(0);
           _pt_min_y = pt_min(1);
           _pt_min_z = pt_min(2);
           _pt_max_x = pt_max(0);
           _pt_max_y = pt_max(1);
           _pt_max_z = pt_max(2);
           _max_x_id = id_max(0);
           _max_y_id = id_max(1);
           _max_z_id = id_max(2);
      };
      
      Cube generateCube( Vector3d pt, const sdf_tools::CollisionMapGrid * collision_map); 
      bool isContains(Cube cube1, Cube cube2);
      pair<Cube, bool> inflateCube(Cube cube, Cube lstcube, const sdf_tools::CollisionMapGrid * collision_map);
      std::vector<Cube> corridorGeneration(const vector<Vector3d> & path_coord, const vector<double> & time, const sdf_tools::CollisionMapGrid * collision_map);
      std::vector<Cube> corridorGeneration(const vector<Vector3d> & path_coord, const sdf_tools::CollisionMapGrid * collision_map);
      void corridorSimplify(vector<Cube> & cubicList);

};

#endif