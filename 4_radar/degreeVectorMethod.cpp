//
// Created by hazyparker on 2021/12/10.
//

#include "degreeVectorMethod.h"
#include "detectBlock.h"
#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

using namespace std;
using namespace cv;

typedef pair<int, int> Pair;

void openLidar(Pair pos) {
    Vector2int grid = getVectorMap("../../maps/test_1.map");
    int pos_i = pos.first;
    int pos_j = pos.second;

    int radius = 30;

    Mat img = imread("../images/test.jpg", IMREAD_COLOR);
    if (img.empty()){
        cout << "loading failed.." << endl;
        exit(100);
    }else{
        cout << "load successfully..." << endl;
    }
    Vec3b unseenColor = {0, 0, 255};
    Vec3b centerColor = {255, 0, 0};
    Vec3b obstacleColor = {0, 255, 0};

    pair<double, double> range;
    vector<pair<double, double>> degreeInterval;
    degreeInterval.emplace_back(0.001, 0.002);
    int i, j;
    for (int r = 1; r <= radius; r++){
        for (i = pos_i - r; i <= pos_i + r; i++){
            for (j = pos_j - r; j <= pos_j +r; j++){
                if (!isValid(grid, {i, j})) continue;
                if (abs(i - pos_i) + abs(j - pos_j) != r || grid[i][j] == 1) continue;

                printf("(%d, %d) is obstacle and being checked...\n", i, j);
                range = getThetaRange(pos, {i, j});
                if (range.second > 314 && range.first < 46 && grid[i][j] == 0){
                    degreeInterval = mergeInterval({0.0001, range.first}, degreeInterval);
                    degreeInterval = mergeInterval({range.second, 359.9999}, degreeInterval);
                    img.at<Vec3b>(i, j) = obstacleColor;
                    continue;
                }
                if (checkInterval(range, degreeInterval)){
                    img.at<Vec3b>(i, j) = unseenColor;
                    printf("(%f, %f) is in blocking area...\n", range.first, range.second);
                }else{
                    img.at<Vec3b>(i, j) = obstacleColor;
                    printf("(%f, %f) is in visible area...\n", range.first, range.second);
                }
                degreeInterval = mergeInterval(range, degreeInterval);
                printf("(%f, %f) added to list...\n", range.first, range.second);
            }



//            // case 1
//            j = pos_j + (r - abs(i - pos_i));
//
//            if (grid[i][j] == 1 || grid[i][pos_j - (r - abs(i - pos_i))] == 1) continue;
//            printf("(%d, %d) is obstacle and being checked...\n", i, j);
//            // get range
//            range = getThetaRange(pos, {i, j});
//            if (checkInterval(range, degreeInterval)){
//                img.at<Vec3b>(i, j) = unseenColor;
//            }else{
//                img.at<Vec3b>(i, j) = obstacleColor;
//            }
//            mergeInterval(range, degreeInterval);
//
//            // case 2
//            j = pos_j - (r - abs(i - pos_i));
//            printf("(%d, %d) is obstacle and being checked...\n", i, j);
//            // get range
//            range = getThetaRange(pos, {i, j});
//            if (checkInterval(range, degreeInterval)){
//                img.at<Vec3b>(i, j) = unseenColor;
//            }else{
//                img.at<Vec3b>(i, j) = obstacleColor;
//            }
//            mergeInterval(range, degreeInterval);



        }
        for (int n = 0; n < degreeInterval.size(); n++){
            cout << degreeInterval[i].first << ", " << degreeInterval[i].second << endl;
        }
    }

    if (degreeInterval.empty()) cout << "damn, list is empty" << endl;
    for (int n = 0; n < degreeInterval.size(); n++){
        cout << degreeInterval[i].first << ", " << degreeInterval[i].second << endl;
    }

    namedWindow("radar", WINDOW_NORMAL);
    img.at<Vec3b>(pos_i, pos_j) = centerColor;
    imshow("radar", img);
    waitKey(1000);

}

vector<pair<double, double>> mergeInterval(const pair<double, double> &newRange, vector<pair<double, double>> &intervals) {
    // https://leetcode-cn.com/problems/merge-intervals/solution/he-bing-qu-jian-by-leetcode-solution/

    intervals.emplace_back(newRange);
    cout << newRange.first << ", to" << newRange.second << endl;

    sort(intervals.begin(), intervals.end());
    vector<pair<double, double>> merged;
    for (auto & interval : intervals) {
        double L = interval.first, R = interval.second;

        if (merged.empty() || merged.back().second < L) {
            merged.emplace_back(L, R);
        }
        else {
            merged.back().second = max(merged.back().second, R);
        }
    }
    return merged;
}

bool checkInterval(const pair<double, double> &newRange, vector<pair<double, double>> &intervals) {
    bool value = false;
    if (intervals.empty()){
        return false;
    }

    sort(intervals.begin(), intervals.end());
    double left = newRange.first;
    double right = newRange.second;

    for (auto & interval : intervals){
        if (left >= interval.first && right <= interval.second){
            value = true;
            break;
        }
    }
    return value;
}

bool isValid(const vector<vector<int>> &grid, const Pair &point){
    // Returns true if row number and column number is in range
    return (point.first >= 0) && (point.first < 100)
        && (point.second >= 0)
        && (point.second < 100);
}