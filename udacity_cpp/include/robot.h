#include <iostream>
#include <cmath>
#include <vector>
#include <random>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
const float PI = 3.141592;

class robot{
    private:
        float x_;
        float y_;
        float orientation_;
        float length_;
        float steering_noise_;
        float distance_noise_;
        float measurement_noise_;
    
    public:
        int num_collisions;
        int num_steps;
        robot(float length);
        ~robot();

    protected:
        void set(float new_x, float new_y ,float new_orientation);
        void set_noise(float str_noise, float dis_noise, float mea_noise);
        bool check_collisions(cv::Mat grid);
        bool check_goal( vector<int> goal, float threshold = 0.2);
        void move(robot next, cv::Mat grid, float steering, float distance,
                  float tolerance = 1.0, float max_steering_angle = PI/4 );
        void sense(vector<float> sense_vec);
        float measurement_prob(vector<float> measurement);
        void repr();

};
