#include <iostream>
#include "./../include/pathplan.h"

plan::plan(Mat grid, vector<int> init, vector<int> goal, float cost):
        cost_(1.0), grid_(grid), row_(grid.rows), col_(grid.cols),
        init_(init), goal_(goal) {}
    
plan::~plan(){}

void plan::make_heuristic(){
    Mat heur = Mat::zeros(row_,col_,CV_32S);

}