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

    int start_x = (int) (ref_pos[0] / 10);
    int start_y = (int) (ref_pos[1] / 10);
    int start_z = (int) (ref_pos[2] / 10);
    
    if (px > 0){
        for (int X = start_x + 1; X < dimX; X++){
            double t = (X - ref_pos[0]) / px;
            if (start_y + py * t >= dimY ||
                start_y + py * t < 0 ||
                start_z + pz * t >= dimZ ||
                start_z + pz * t < 0){
                break;
                
            }

            grid[X + dimX * ( (int)(start_y + py * t) + dimY * (int)(start_z + pz * t) )]++;
        }
    }
    else {
        for (int X = start_x - 1; X < dimX; X--){
            double t = ray_hit(X, ref_pos[0], px);
            if (in_bound(start_x, start_y, start_z, px, py, pz, t)){
                brake;
            }

            grid[start_x + int (py * t) + dimX * ( (int)(start_y + py * t) + dimY * (int)(start_z + pz * t) )]++;

    }

}



double Voxel::ray_hit(int plain, double pos, double delta){
    return (plain - pos) / delta;
}

bool Voxel::in_bound(int x, int y, int z, double px, double py, double pz, double t){
    if (x + py * t >= dimX ||
        x + py * t < 0 ||
        y + py * t >= dimY ||
        y + py * t < 0 ||
        z + pz * t >= dimZ ||
        z + pz * t < 0){
        return false;
    }
    return true;
                
}
} //namespace
