#include "./../include/control.h"

void run(float length, Mat grid, vector<int> goal, vector<vector<float>> spath, vector<float> noise,
        vector<float> params, bool printflag, float speed, int time){
    Gnuplot g1("ff");
    for(int i=0; i<grid.rows; i++){
        for(int j=0; j<grid.cols; j++){
            if(grid.at<int>(i,j) == 1){

            }
        }
    }

    robot myrobot = robot(length);
    myrobot.set(0.0,0.0,0.0);
    myrobot.set_noise(noise[0],noise[1],noise[2]);
    particles filter = particles(length, myrobot.x_, myrobot.y_, myrobot.orientation_, noise[0],noise[1],noise[2]);

    float cte = 0.0;
    float err = 0.0;
    int N = 0;
    int index = 0;

    float diff_cte, dx, dy, rx, ry, u, steer;
    vector<float> estimate, Z;

    while(!(myrobot.check_goal(goal) && N<time  && index<spath.size()-1)){
        
        diff_cte = -cte;
        estimate = filter.get_position();

        dx = spath[index+1][0] - spath[index][0];
        dy = spath[index+1][1] - spath[index][1];
        rx = estimate[0] - spath[index][0];
        ry = estimate[1] - spath[index][1];

        u = (rx*dx + ry*dy) / (dx*dx + dy*dy);
        cte = (ry*dx - rx*dy) / (dx*dx + dy*dy);

        if(u>1.0){
            index += 1;
        }
        diff_cte += cte;
        steer = -params[0] * cte - params[1] * diff_cte;
        myrobot = myrobot.move(grid,steer,speed);
        filter.move(grid,steer,speed);

        Z = myrobot.sense();
        filter.sense(Z);
        myrobot.check_collisions(grid);

        err += (cte*cte);
        N += 1;
    }

}