//
// Created by hazyparker on 2021/12/10.
//

#ifndef INC_4_RADAR_DEGREEVECTORMETHOD_H
#define INC_4_RADAR_DEGREEVECTORMETHOD_H


#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

using namespace std;
using namespace cv;

typedef pair<int, int> Pair;

void openLidar(Pair pos);
vector<pair<double, double>> mergeInterval(const pair<double, double> &newRange,
                                           vector<pair<double, double>> &intervals);
bool checkInterval(const pair<double, double> &newRange,
                   vector<pair<double, double>> &intervals);

bool isValid(const vector<vector<int>> &grid, const Pair &point);


#endif //INC_4_RADAR_DEGREEVECTORMETHOD_H
