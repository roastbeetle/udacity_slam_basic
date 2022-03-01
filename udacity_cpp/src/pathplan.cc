#include <iostream>
#include "./../include/pathplan.h"

plan::plan(Mat grid, vector<int> init, vector<int> goal, float cost):
        cost_(1.0), grid_(grid), row_(grid.rows), col_(grid.cols),
        init_(init), goal_(goal) {}
    
plan::~plan(){}

void plan::make_heuristic(){
    heur_ = Mat::zeros(row_, col_,CV_32S);
    for(int i = 0; i< row_; i++){
        for(int j = 0; j<col_; j++){
            heur_.at<int>(i,j) = abs(i-goal_[0])+abs(j-goal_[1]);
        }
    }

}
void plan::astar(){
    if(heur_.empty())
        cout<< "error" <<endl;
    
    vector<int> up    = {-1, 0};
    vector<int> down  = { 1, 0};
    vector<int> left  = { 0,-1};
    vector<int> right = { 0, 1};

    Mat closed = Mat::zeros(row_,col_,CV_32S);
    Mat action = Mat::zeros(row_,col_,CV_32S);
    closed.at<int>(init_[0],init_[1]) = 1;

    int x = init_[0];
    int y = init_[1];
    int h = heur_.at<int>(x,y);
    int g = 0.0;
    int f = g + h;
    
    vector<int> open_e = {f,g,h,x,y};
    vector<vector<int>> open = {open_e};

    bool found  = false;
    bool resign = false;
    int  count  = 0;

    while(!found && !resign){
        if(open.empty()){
            resign = true;
            cout<<"Search Terminated Failed"<<endl;
        }
        else{
            open.
        }
    }

}