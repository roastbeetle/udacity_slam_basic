#ifndef CONTROL_H
#define CONTROL_H

#include "./../include/particlefilter.h"
#include <gnuplot-iostream.h>

void run(float length, Mat grid, vector<int> goal, vector<vector<float>> spath, vector<float> noise,
        vector<float> params, bool printflag = true, float speed = 0.1, int time = 1000);

#endif