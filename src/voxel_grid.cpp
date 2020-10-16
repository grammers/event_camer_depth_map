#include <voxel_grid.hpp>

namespace GRID{

Voxel::Voxel(int dimX, int dimY, int dimZ){
    this->dimX = dimX;
    this->dimY = dimY;
    this->dimZ = dimZ;

    init_pos[0] = dimX / 2;
    init_pos[1] = dimY / 2;
    init_pos[2] = dimZ / 2;
    init_pos[3] = 0;
    init_pos[4] = 0;
    init_pos[5] = 0;
    
    grid.resize(dimX * dimY * dimZ);

    std::fill(grid.begin(), grid.end(), 0.f);

}


void Voxel::add_ray(double *cam_pos, double *event_dir){
    double ref_pos [6];
    ref_pos[0] = init_pos[0] + cam_pos[0];
    ref_pos[1] = init_pos[1] + cam_pos[1];
    ref_pos[2] = init_pos[2] + cam_pos[2];
    ref_pos[3] = init_pos[3] + cam_pos[3];
    ref_pos[4] = init_pos[4] + cam_pos[4] + atan(event_dir[2] / event_dir[3]);
    ref_pos[5] = init_pos[5] + cam_pos[5] + atan(event_dir[1] / event_dir[3]);

    double px = cos(ref_pos[5]) * cos(ref_pos[4]);
    double py = sin(ref_pos[5]) * cos(ref_pos[4]);
    double pz = sin(ref_pos[4]);

    int start_x = (int) (ref_pos[0] * 10);
    int start_y = (int) (ref_pos[1] * 10);
    int start_z = (int) (ref_pos[2] * 10);
    
    int change = ray_direction(px);
    for (int X = start_x + change; X < dimX; X++){
        double t = (X - ref_pos[0]) / px;
        if (add_hit(t, start_x, start_y, start_z, px, py, pz)){
            break;
        }
    }

    change = ray_direction(py);
    for (int Y = start_y + change; Y < dimY; Y++){
        double t = (Y - ref_pos[1]) / py;
        if (add_hit(t, start_x, start_y, start_z, px, py, pz)){
            break;
        }
    }

    change = ray_direction(pz);
    for (int Z = start_z + change; Z < dimZ; Z++){
        double t = (Z - ref_pos[2]) / pz;
        if (add_hit(t, start_x, start_y, start_z, px, py, pz)){
            break;
        }
    }

}

int Voxel::ray_direction(double p){
    if (p > 0) {
        return 1;
    return -1;
}

bool Voxel::add_hit(double t, int start_x, int start_y, int start_z,
                        double px, double py, double pz){

    int x = start_x + (int) (px * t);
    int y = start_y + (int) (py * t);
    int z = start_z + (int) (pz * t);

    if (in_bound(x, y, z){
        return false;
    }
    
    grid[x + dimX * (y + dimY * z)]++;

    return true;
}


bool Voxel::in_bound(int x, int y, int z){
    if (x >= dimX || x < 0 ||
        y >= dimY || y < 0 ||
        z >= dimZ || z < 0){

        return false;
    }
    return true;
                
}
} //namespace
