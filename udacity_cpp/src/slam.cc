#include "control.cc"
#include "./../include/pathplan.h"

using namespace std;

float steering_noise, distance_noise, measurement_noise;

vector<double> noise = {steering_noise, distance_noise, measurement_noise };

void slam(float length, Mat grid, vector<int> init, vector<int> goal, vector<float> noise,
                   float weight_data, float weigth_smooth, float p_gain, float d_gain){
    plan path = plan(grid,init,goal);
    path.astar();
    path.smooth(weight_data, weigth_smooth);
    return run(length,grid,goal,path.spath_,noise,{p_gain,d_gain});
}

int main(){
    return 0;
};