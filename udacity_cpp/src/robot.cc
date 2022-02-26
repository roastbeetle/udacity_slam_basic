#include <iostream>
#include "./../include/robot.h"

using namespace std;

robot::robot(float length):
        x_(0.0), y_(0.0), orientation_(0.0), length_(length),
        steering_noise_(0.0), distance_noise_(0.0), measurement_noise_(0.0),
        num_collisions(0), num_steps(0) {}
robot::~robot(){}

void robot::set(float new_x, float new_y ,float new_orientation){
    x_ = new_x;
    y_ = new_y;
    orientation_= fmod(new_orientation,(2.0*PI));
}
void robot::set_noise(float str_noise, float dis_noise, float mea_noise){
    steering_noise_ = str_noise;
    distance_noise_ = dis_noise;
    measurement_noise_ = mea_noise;
}

bool robot::check_collisions(cv::Mat grid){
    for(int i = 0; i< grid.cols; i++){
        for(int j = 0; j< grid.rows; j++){
            if(grid.at<int>(i,j) == 1){
                float dist = sqrt( (x_ - float(i))*(x_ - float(i))
                                  +(y_ - float(j))*(y_ - float(j)));
                if(dist < 0.5){
                    num_collisions = num_collisions + 1;
                    return false;
                }

            }
        
        }
    }
    return true; 
}

bool robot::check_goal( vector<int> goal, float threshold = 0.2){
    float dist = sqrt((goal[0]-x_)*(goal[0]-x_)
                     +(goal[1]-y_)*(goal[1]-y_));
    return dist < threshold;
}

void robot::move(cv::Mat grid, float steering, float distance, float tolerance = 1.0, float max_steering_angle = PI/4 ){
    if(steering > max_steering_angle)
        steering = max_steering_angle;   
    if(steering < -max_steering_angle)
        steering = -max_steering_angle;
    if(distance < 0.0)
        distance = 0.0;
    
    robot::robot next(length_);
    next.set_noise(steering_noise_, distance_noise_, measurement_noise_);
    next.num_collisions = num_collisions;
    next.num_steps = num_steps+1;

    normal_distribution<float> distribution(steering,steering_noise_);
}