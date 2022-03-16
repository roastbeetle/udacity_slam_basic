#include <iostream>
#include "control.h"
#include "pathplan.h"
#include "config.h"

using namespace cv;

float steering_noise, distance_noise, measurement_noise;
vector<double> noise = {steering_noise, distance_noise, measurement_noise };

void slam(float length, Mat grid, vector<int> init, vector<int> goal, vector<float> noise,
                   float weight_data, float weigth_smooth, float p_gain, float d_gain){
    plan path = plan(grid,init,goal);
    path.make_heuristic();
    //cout<<"bug"<<endl;
    path.astar();
    //cout<<"bug"<<endl;
    path.smooth(weight_data, weigth_smooth);
    //cout<<"bug"<<endl;
    vector<int> res = run(length,grid,goal,path.spath_,noise,{p_gain,d_gain});
    cout<<"goal:"<<res[0]<<" collision:"<<res[1]<<" step:"<<res[2]<<endl;
}

int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: parameter_file"<<endl;
        return 1;
    }
   int data[] ={0, 0, 0, 1, 0, 1,
                0, 1, 0, 1, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 1, 0, 1, 1, 0,
                0, 0, 0, 1, 0, 0};

    Mat grid(5,6,CV_32S,data);
    Config::setParameterFile ( argv[1] );
    float length = Config::get<float>("robot_length");

    int init_x = Config::get<int>("init_x");
    int init_y = Config::get<int>("init_y");
    int goal_x = Config::get<int>("goal_x");
    int goal_y = Config::get<int>("goal_y");

    float steering_noise = Config::get<float>("steering_noise");
    float distance_noise = Config::get<float>("distance_noise");
    float measurement_noise = Config::get<float>("measurement_noise");

    float weight_data = Config::get<float>("weight_data");
    float weight_smooth = Config::get<float>("weight_smooth");
    float p_gain = Config::get<float>("p_gain");
    float d_gain = Config::get<float>("d_gain");

    slam(length, grid, {init_x,init_y}, {goal_x,goal_y},
        {steering_noise,distance_noise,measurement_noise},
        weight_data,weight_smooth,p_gain,d_gain);

    return 0;
};