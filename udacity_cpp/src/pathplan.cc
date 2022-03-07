#include <iostream>
#include "./../include/pathplan.h"


plan::plan(Mat grid, vector<int> init, vector<int> goal, float cost):
        cost_(cost), grid_(grid), row_(grid.rows), col_(grid.cols),
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
    vector<vector<int>> delta= {{-1, 0},
                                { 0,-1},
                                { 1, 0},
                                { 0, 1}};

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
            sort(open.begin(), open.end());
            vector<int> next = open.back();
            open.pop_back();
            g = next[1];
            x = next[3];
            y = next[4];
        }
        if(x==goal_[0] && y ==goal_[1]){
            found = true;
        }
        else{
            for(int i = 0; i< delta.size() ; i++){
                int x2 = x + delta[i][0];
                int y2 = y + delta[i][1];
                if( x2>=0 && x2<row_ && y2>=0 && y2<col_ ){
                    if( closed.at<int>(x2,y2) == 0 && grid_.at<int>(x2,y2) == 0){
                        int g2 = g + cost_;
                        int h2 = heur_.at<int>(x2,y2);
                        int f2 = g2 + h2;
                        open.push_back({f2,g2,h2,x2,y2});
                        closed.at<int>(x2,y2) = 1;
                        action.at<int>(x2,y2) = 1;
                    }
                }
            }
        }
        count = count + 1;
    }
    
    vector<vector<int>> invpath;
    x = goal_[0]; 
    y = goal_[1];
    invpath.push_back({x,y});
    while( x != init_[0] or y != init_[1] ){
        int x2 = x - delta[action.at<int>(x,y)][0];
        int y2 = y - delta[action.at<int>(x,y)][1];
        x = x2;
        y = y2;
        invpath.push_back({x,y});
    } 

    path_.clear();
    for(int i=0; i<invpath.size(); i++){
        path_.push_back(invpath[invpath.size()-1-i]);
    }
}

void plan::smooth(float weight_data, float weight_smooth, float tolerance){
    if(path_.empty())
        cout<<"Search Terminated Failed"<<endl;
    
    for(int i=0; i<path_.size();i++){
        spath_.push_back({static_cast<float>(path_[i][0]), 
                          static_cast<float>(path_[i][1])});
    }

    float change = tolerance;
    while(change>=tolerance){
        change = 0.0;
        for(int i=1; i<path_.size()-1;i++){
            for(int j=0; j<2; j++){
                float aux = spath_[i][j];
                spath_[i][j] += weight_data*(path_[i][j] - spath_[i][j]);
                spath_[i][j] += weight_smooth*(spath_[i-1][j] + spath_[i+1][j] - (2.0 * spath_[i][j]));
                    
                if(i >= 2){
                    spath_[i][j] += 0.5 * weight_smooth*
                            (2.0 * spath_[i-1][j] - spath_[i-2][j] - spath_[i][j]);
                }
                if( i <= path_.size() - 3){
                    spath_[i][j] += 0.5 * weight_smooth*
                            (2.0 * spath_[i+1][j] - spath_[i+2][j] - spath_[i][j]);
                }
                change += abs(aux - spath_[i][j]);
            }
        }
    }
            
}