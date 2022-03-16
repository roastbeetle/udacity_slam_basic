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

bool robot::check_goal( vector<int> goal, float threshold){
    float dist = sqrt((goal[0]-x_)*(goal[0]-x_)
                     +(goal[1]-y_)*(goal[1]-y_));
    return dist < threshold;
}

robot robot::move(cv::Mat grid, float steering, float distance, float tolerance, float max_steering_angle){
    if(steering > max_steering_angle)
        steering = max_steering_angle;   
    if(steering < -max_steering_angle)
        steering = -max_steering_angle;
    if(distance < 0.0)
        distance = 0.0;
    
    robot next = robot(length_);
    next.set_noise(steering_noise_, distance_noise_, measurement_noise_);
    next.num_collisions = num_collisions;
    next.num_steps = num_steps+1;

    random_device rd;
    mt19937 gen(rd());
    normal_distribution<float> d_s(steering,steering_noise_);
    normal_distribution<float> d_d(distance,distance_noise_);

    float steering2 = d_s(gen);
    float distance2 = d_d(gen);
    float turn = tan(steering2)*distance2 / length_;
    
    if(abs(turn) < tolerance){
        next.set(x_+ (distance2*cos(orientation_)),
                 y_+ (distance2*sin(orientation_)),
                 fmod(orientation_+turn,(2.0*PI)));
    }
    else{
        float radius = distance2/turn;
        float cx = x_ -(sin(orientation_)*radius);
        float cy = y_ +(cos(orientation_)*radius);
        float cori = fmod(orientation_+turn,(2.0*PI));
        next.set(cx+(sin(cori)*radius),cy-(cos(cori)*radius),cori);
    }
    return next;
}

vector<float> robot::sense(){
    vector<float> sense_vec;
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<float> d_x(x_,measurement_noise_);
    normal_distribution<float> d_y(y_,measurement_noise_);
    sense_vec.push_back(d_x(gen));
    sense_vec.push_back(d_y(gen));
    return sense_vec;
}

float robot::measurement_prob(vector<float> measurement){
    float erx = measurement[0] - x_;
    float ery = measurement[1] - y_;
    float mn2 = measurement_noise_*measurement_noise_;
    
    float error = exp(-(erx*erx)/(mn2*0.5)/sqrt(2.0*PI*mn2));
    error = error* exp(-(ery*ery)/(mn2*0.5)/sqrt(2.0*PI*mn2));
    return error;
}   

void robot::repr(){
    cout<<"["<< x_ << y_ << "]" << endl;
}
