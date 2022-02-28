#include <iostream>
#include <cmath>
#include <vector>
#include <random>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
const float PI = 3.141592;

class plan{
    private:
        float cost_;
        Mat grid_;
        Mat heur_;
        int row_;
        int col_;
        vector<int> init_;
        vector<int> goal_;
    
    public:
        plan(Mat grid, vector<int> init, vector<int> goal, float cost);
        ~plan();

    protected:
        void make_heuristic();
        void astar();
        void smooth(float weight_data = 0.1, float weight_smooth = 0.1, float tolerance = 0.000001);


};