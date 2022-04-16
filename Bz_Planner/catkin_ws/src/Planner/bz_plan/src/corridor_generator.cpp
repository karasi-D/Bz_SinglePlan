#include "data_type.h"
using namespace std;    
using namespace Eigen;
using namespace sdf_tools;

Cube CorridorGenerate::generateCube( Vector3d pt, const sdf_tools::CollisionMapGrid * collision_map) 
{   
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
    Cube cube;
    
    pt(0) = max(min(pt(0), _pt_max_x), _pt_min_x);
    pt(1) = max(min(pt(1), _pt_max_y), _pt_min_y);
    pt(2) = max(min(pt(2), _pt_max_z), _pt_min_z);

    Vector3i pc_index = collision_map->LocationToGridIndex(pt);    
    Vector3d pc_coord = collision_map->GridIndexToLocation(pc_index);

    cube.center = pc_coord;
    double x_u = pc_coord(0);
    double x_l = pc_coord(0);
    
    double y_u = pc_coord(1);
    double y_l = pc_coord(1);
    
    double z_u = pc_coord(2);
    double z_l = pc_coord(2);

    cube.vertex.row(0) = Vector3d(x_u, y_l, z_u);  
    cube.vertex.row(1) = Vector3d(x_u, y_u, z_u);  
    cube.vertex.row(2) = Vector3d(x_l, y_u, z_u);  
    cube.vertex.row(3) = Vector3d(x_l, y_l, z_u);  

    cube.vertex.row(4) = Vector3d(x_u, y_l, z_l);  
    cube.vertex.row(5) = Vector3d(x_u, y_u, z_l);  
    cube.vertex.row(6) = Vector3d(x_l, y_u, z_l);  
    cube.vertex.row(7) = Vector3d(x_l, y_l, z_l);  

    return cube;
}

bool CorridorGenerate::isContains(Cube cube1, Cube cube2)
{   
    if( cube1.vertex(0, 0) >= cube2.vertex(0, 0) && cube1.vertex(0, 1) <= cube2.vertex(0, 1) && cube1.vertex(0, 2) >= cube2.vertex(0, 2) &&
        cube1.vertex(6, 0) <= cube2.vertex(6, 0) && cube1.vertex(6, 1) >= cube2.vertex(6, 1) && cube1.vertex(6, 2) <= cube2.vertex(6, 2)  )
        return true;
    else
        return false; 
}

pair<Cube, bool> CorridorGenerate::inflateCube(Cube cube, Cube lstcube, const sdf_tools::CollisionMapGrid * collision_map)
{   
    Cube cubeMax = cube;

    // Inflate sequence: left, right, front, back, below, above                                                                                
    MatrixXi vertex_idx(8, 3);
    for (int i = 0; i < 8; i++)
    { 
        double coord_x = max(min(cube.vertex(i, 0), _pt_max_x), _pt_min_x);
        double coord_y = max(min(cube.vertex(i, 1), _pt_max_y), _pt_min_y);
        double coord_z = max(min(cube.vertex(i, 2), _pt_max_z), _pt_min_z);
        Vector3d coord(coord_x, coord_y, coord_z);

        Vector3i pt_idx = collision_map->LocationToGridIndex(coord);

        if( collision_map->Get( (int64_t)pt_idx(0), (int64_t)pt_idx(1), (int64_t)pt_idx(2) ).first.occupancy > 0.5 )
        {       
            ROS_ERROR("[Planning Node] path has node in obstacles !");
            return make_pair(cubeMax, false);
        }
        
        vertex_idx.row(i) = pt_idx;
    }

    int id_x, id_y, id_z;

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

    bool collide;
    MatrixXi vertex_idx_lst = vertex_idx;

    int iter = 0;
    while(iter < _max_inflate_iter)
    { 
        // Y- now is the left side : (p1 -- p4 -- p8 -- p5) face sweep
        // ############################################################################################################
         
        int y_lo = max(0, vertex_idx(0, 1) - _step_length);
        int y_up = min(_max_y_id, vertex_idx(1, 1) + _step_length);

        collide  = false;
        for(id_y = vertex_idx(0, 1); id_y >= y_lo; id_y-- )
        {   
            if( collide == true) 
                break;
            
            for(id_x = vertex_idx(0, 0); id_x >= vertex_idx(3, 0); id_x-- )
            {    
                if( collide == true) 
                    break;

                for(id_z = vertex_idx(0, 2); id_z >= vertex_idx(4, 2); id_z-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(0, 1) = min(id_y+2, vertex_idx(0, 1));
            vertex_idx(3, 1) = min(id_y+2, vertex_idx(3, 1));
            vertex_idx(7, 1) = min(id_y+2, vertex_idx(7, 1));
            vertex_idx(4, 1) = min(id_y+2, vertex_idx(4, 1));
        }
        else
            vertex_idx(0, 1) = vertex_idx(3, 1) = vertex_idx(7, 1) = vertex_idx(4, 1) = id_y + 1;
        
        // Y+ now is the right side : (p2 -- p3 -- p7 -- p6) face
        // ############################################################################################################
        collide = false;
        for(id_y = vertex_idx(1, 1); id_y <= y_up; id_y++ )
        {   
            if( collide == true) 
                break;
            
            for(id_x = vertex_idx(1, 0); id_x >= vertex_idx(2, 0); id_x-- )
            {
                if( collide == true) 
                    break;

                for(id_z = vertex_idx(1, 2); id_z >= vertex_idx(5, 2); id_z-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(1, 1) = max(id_y-2, vertex_idx(1, 1));
            vertex_idx(2, 1) = max(id_y-2, vertex_idx(2, 1));
            vertex_idx(6, 1) = max(id_y-2, vertex_idx(6, 1));
            vertex_idx(5, 1) = max(id_y-2, vertex_idx(5, 1));
        }
        else
            vertex_idx(1, 1) = vertex_idx(2, 1) = vertex_idx(6, 1) = vertex_idx(5, 1) = id_y - 1;

        // X + now is the front side : (p1 -- p2 -- p6 -- p5) face
        // ############################################################################################################
        int x_lo = max(0, vertex_idx(3, 0) - _step_length);
        int x_up = min(_max_x_id, vertex_idx(0, 0) + _step_length);

        collide = false;
        for(id_x = vertex_idx(0, 0); id_x <= x_up; id_x++ )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(0, 1); id_y <= vertex_idx(1, 1); id_y++ )
            {
                if( collide == true) 
                    break;

                for(id_z = vertex_idx(0, 2); id_z >= vertex_idx(4, 2); id_z-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(0, 0) = max(id_x-2, vertex_idx(0, 0)); 
            vertex_idx(1, 0) = max(id_x-2, vertex_idx(1, 0)); 
            vertex_idx(5, 0) = max(id_x-2, vertex_idx(5, 0)); 
            vertex_idx(4, 0) = max(id_x-2, vertex_idx(4, 0)); 
        }
        else
            vertex_idx(0, 0) = vertex_idx(1, 0) = vertex_idx(5, 0) = vertex_idx(4, 0) = id_x - 1;    

        // X- now is the back side : (p4 -- p3 -- p7 -- p8) face
        // ############################################################################################################
        collide = false;
        for(id_x = vertex_idx(3, 0); id_x >= x_lo; id_x-- )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(3, 1); id_y <= vertex_idx(2, 1); id_y++ )
            {
                if( collide == true) 
                    break;

                for(id_z = vertex_idx(3, 2); id_z >= vertex_idx(7, 2); id_z-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(3, 0) = min(id_x+2, vertex_idx(3, 0)); 
            vertex_idx(2, 0) = min(id_x+2, vertex_idx(2, 0)); 
            vertex_idx(6, 0) = min(id_x+2, vertex_idx(6, 0)); 
            vertex_idx(7, 0) = min(id_x+2, vertex_idx(7, 0)); 
        }
        else
            vertex_idx(3, 0) = vertex_idx(2, 0) = vertex_idx(6, 0) = vertex_idx(7, 0) = id_x + 1;

        // Z+ now is the above side : (p1 -- p2 -- p3 -- p4) face
        // ############################################################################################################
        collide = false;
        int z_lo = max(0, vertex_idx(4, 2) - _step_length);
        int z_up = min(_max_z_id, vertex_idx(0, 2) + _step_length);
        for(id_z = vertex_idx(0, 2); id_z <= z_up; id_z++ )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(0, 1); id_y <= vertex_idx(1, 1); id_y++ )
            {
                if( collide == true) 
                    break;

                for(id_x = vertex_idx(0, 0); id_x >= vertex_idx(3, 0); id_x-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(0, 2) = max(id_z-2, vertex_idx(0, 2));
            vertex_idx(1, 2) = max(id_z-2, vertex_idx(1, 2));
            vertex_idx(2, 2) = max(id_z-2, vertex_idx(2, 2));
            vertex_idx(3, 2) = max(id_z-2, vertex_idx(3, 2));
        }
        vertex_idx(0, 2) = vertex_idx(1, 2) = vertex_idx(2, 2) = vertex_idx(3, 2) = id_z - 1;

        // Z- now is the below side : (p5 -- p6 -- p7 -- p8) face
        // ############################################################################################################
        collide = false;
        for(id_z = vertex_idx(4, 2); id_z >= z_lo; id_z-- )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(4, 1); id_y <= vertex_idx(5, 1); id_y++ )
            {
                if( collide == true) 
                    break;

                for(id_x = vertex_idx(4, 0); id_x >= vertex_idx(7, 0); id_x-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(4, 2) = min(id_z+2, vertex_idx(4, 2));
            vertex_idx(5, 2) = min(id_z+2, vertex_idx(5, 2));
            vertex_idx(6, 2) = min(id_z+2, vertex_idx(6, 2));
            vertex_idx(7, 2) = min(id_z+2, vertex_idx(7, 2));
        }
        else
            vertex_idx(4, 2) = vertex_idx(5, 2) = vertex_idx(6, 2) = vertex_idx(7, 2) = id_z + 1;

        if(vertex_idx_lst == vertex_idx)
            break;

        vertex_idx_lst = vertex_idx;

        MatrixXd vertex_coord(8, 3);
        for(int i = 0; i < 8; i++)
        {   
            int index_x = max(min(vertex_idx(i, 0), _max_x_id - 1), 0);
            int index_y = max(min(vertex_idx(i, 1), _max_y_id - 1), 0);
            int index_z = max(min(vertex_idx(i, 2), _max_z_id - 1), 0);

            Vector3i index(index_x, index_y, index_z);
            Vector3d pos = collision_map->GridIndexToLocation(index);
            vertex_coord.row(i) = pos;
        }

        cubeMax.setVertex(vertex_coord, _resolution);
        if( isContains(lstcube, cubeMax))        
            return make_pair(lstcube, false); // 和上一个cube是包含关系

        iter ++;
    }

    return make_pair(cubeMax, true);
}

std::vector<Cube> CorridorGenerate::corridorGeneration(const vector<Vector3d> & path_coord, const vector<double> & time, const sdf_tools::CollisionMapGrid * collision_map)
{   
    vector<Cube> cubeList;
    Vector3d pt;

    Cube lstcube;

    for (int i = 0; i < (int)path_coord.size(); i += 1)
    {
        pt = path_coord[i];

        Cube cube = generateCube(pt, collision_map);
        auto result = inflateCube(cube, lstcube, collision_map);

        if(result.second == false)
            continue;

        cube = result.first;
        
        lstcube = cube;
        cube.t = time[i];
        cubeList.push_back(cube);
    }
    return cubeList;
}

std::vector<Cube> CorridorGenerate::corridorGeneration(const vector<Vector3d> & path_coord, const sdf_tools::CollisionMapGrid * collision_map)
{   
    vector<Cube> cubeList;
    Vector3d pt;

    Cube lstcube;

    for (int i = 0; i < (int)path_coord.size(); i += 1)
    {
        pt = path_coord[i];

        Cube cube = generateCube(pt, collision_map);
        auto result = inflateCube(cube, lstcube, collision_map);

        if(result.second == false)
            continue;

        cube = result.first;
        
        lstcube = cube;
        cubeList.push_back(cube);
    }
    return cubeList;
}

void CorridorGenerate::corridorSimplify(vector<Cube> & cubicList)
{
    vector<Cube> cubicSimplifyList;
    for(int j = (int)cubicList.size() - 1; j >= 0; j--)
    {   
        for(int k = j - 1; k >= 0; k--)
        {   
            if(cubicList[k].valid == false)
                continue;
            else if(isContains(cubicList[j], cubicList[k]))
                cubicList[k].valid = false;   
        }
    }

    for(auto cube:cubicList)
        if(cube.valid == true)
            cubicSimplifyList.push_back(cube);

    cubicList = cubicSimplifyList;
}