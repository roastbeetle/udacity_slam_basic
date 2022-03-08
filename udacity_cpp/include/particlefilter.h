#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include "robot.h"

using namespace std;
using namespace cv;

class particles{
    private:
        int N_;
        float steering_noise_;
        float distance_noise_;
        float measurement_noise_;
        vector<robot> data_;
    
    public:
        vector<float> get_position();
        void move(Mat grid, float steer, float speed);
        void sense(vector<float> Z);
        particles(float length, float x, float y, float theta, float steering_noise, float distance_noise, float measurement_noise, int N=100 );
        ~particles();

};

#endif