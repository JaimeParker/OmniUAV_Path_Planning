//
// Created by hazyparker on 2021/12/8.
//

#ifndef INC_4_RADAR_DETECTBLOCK_H
#define INC_4_RADAR_DETECTBLOCK_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>

using namespace std;
using namespace cv;

typedef vector<vector <int>> Vector2int;
typedef pair<int, int> Pair;

Vector2int getVectorMap(const char* filepath);

void showDetectBlocks(int i, int j);

void drawDetectedBlocks(Vector2int &grid, const Pair &center);

pair<double, double> getThetaRange(Pair point_0, Pair point_d);

bool isValid(Pair point);

bool isOnEdge(Pair point, Pair center, int r);

#endif //INC_4_RADAR_DETECTBLOCK_H
