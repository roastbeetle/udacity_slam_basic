#include "./../include/particlefilter.h"

particles::particles(float length, float x, float y, float theta, float steering_noise, float distance_noise, float measurement_noise, int N ):
                    steering_noise_(steering_noise), distance_noise_(distance_noise), measurement_noise_(measurement_noise), N_(100) {
    for(int i = 0;i<N_;i++){
        robot r = robot(length);
        r.set(x,y,theta);
        r.set_noise(steering_noise_,distance_noise_,measurement_noise_);
        data_.push_back(r);
    }
    
}
particles::~particles(){}

vector<float> particles::get_position(){
    float x = 0.0;
    float y = 0.0;
    float orientation = 0.0;

    for(int i=0; i<N_; i++){
        x += data_[i].x_;
        y += data_[i].y_;
        orientation += fmod((data_[i].orientation_ - data_[0].orientation_ + PI),(2.0 * PI)) + data_[0].orientation_ - PI;
    }
    return {x/N_, y/N_,orientation/N_};
}

void particles::move(Mat grid, float steer, float speed){
    vector<robot> newdata;
    for(int i=0; i<N_; i++){
        robot update = data_[i].move(grid,steer,speed);
        newdata.push_back(update);
    }
    data_ = newdata;
}

void particles::sense(vector<float> Z){
    vector<float> w;
    vector<robot> p3;
    for(int i=0; i<N_; i++)
        w.push_back(data_[i].measurement_prob(Z));

    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> d_i(0,N_);
    uniform_real_distribution<float> d_f(0.0, 1.0);

    int index = d_i(gen);
    float fran = d_f(gen);

    float beta = 0.0;
    float maxw = *max_element(w.begin(), w.end()); ;
    for(int i=0; i<N_; i++){
        beta += fran * 2.0 * maxw;
        while(beta>w[index]){
            beta -= w[index];
            index = fmod(index+1,N_);
        }
        p3.push_back(data_[index]);
    }
    data_ = p3;
        
}