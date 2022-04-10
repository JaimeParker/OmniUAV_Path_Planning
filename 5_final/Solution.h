//
// Created by hazyparker on 2021/12/10.
//

#ifndef INC_5_FINAL_SOLUTION_H
#define INC_5_FINAL_SOLUTION_H
#include "cmath"
#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <queue>
#include <set>
#include <stack>
#include <tuple>
#include <vector>
#include <utility>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

typedef pair<int, int> Pair;
typedef vector<vector<int>> Vec2int;
typedef tuple<bool, int, int, int> PathVar;

class Solution {
private:
    int ROW;
    int COL;

    Pair StartPoint;
    Pair EndPoint;
    stack<Pair> DynamicPath;
    // FIXME: cannot define vector in class? like vector<int> vec; have to be vec = vector<int>(100)
    // reference:
    // https://stackoverflow.com/questions/70332332/how-to-use-vector-as-a-private-member-in-class-to-store-and-pass-data/70332667#70332667
    // however, it's still not fixed

    Vec2int map_vector;
    Vec2int radar_vector;
    Vec2int check_vector;
    Vec2int show_vector;

public:
    Solution():
            ROW{ 100 },
            COL{ 100 },
            map_vector(ROW, vector<int>(COL)),
            radar_vector(ROW, vector<int>(COL)),
            check_vector(ROW, vector<int>(COL)),
            show_vector(ROW, vector<int>(COL)),
            StartPoint({10, 10}),
            EndPoint({90, 90})
    {}
    // A structure to hold the necessary parameters
    struct cell {
        // Row and Column index of its parent
        Pair nodeParent;
        // f = g + h
        double cost_f, cost_g, cost_h;
        cell():
                nodeParent(-1, -1), cost_f(-1), cost_g(-1), cost_h(-1){}
    };


    /**
     * set start point in coordinate position, then trans to array pos
     * @param x corPos x
     * @param y corPos y
     */
    void setStartPoint(int x, int y);

    /**
     * set end point in coordinate position, then trans to array pos
     * @param x corPos x
     * @param y corPos y
     */
    void setEndPoint(int x, int y);

    /**
     * transfer coordinate position to (i, j) display in array which starting at [0]
     * @param corPos pair(STL, C++) coordinate position of a point
     * @return array position
     */
    static Pair cor2ArrayPos(Pair corPos);

    /**
     * check whether given cell (row, col) is a valid cell or not.
     * @param grid
     * @param point
     * @return bool
     */
    bool isValid(const vector<vector <int>> &grid, const Pair& point) const;

    /**
     * A Utility Function to check whether the given cell is blocked or not
     * @param grid
     * @param point
     * @return
     */
    bool isUnBlocked(const vector<vector <int>> &grid, const Pair& point) const;

    /**
     * A Utility Function to check whether destination cell has been reached or not
     * @param position
     * @param dest
     * @return
     */
    bool isDestination(const Pair& position);

    /**
     * A Utility Function to calculate the 'h' heuristics
     * @param src
     * @param dest
     * @return
     */
    static double calculateHValue(const Pair& src, const Pair& dest);

    /**
     * transfer vector<vector<int>> to gray scale Mat(image)
     * @param grid
     * @return Mat mat
     */
    Mat vector2ObstacleMap(Vec2int &grid) const;

    /**
     * read m_map array from file(mapName), write it in map_vector
     * @param mapName
     * @return map_vector
     */
    Vec2int getVectorRawGrid(const string& mapName);

    /**
     * using erode function in openCV to add boundaries
     * @param grid
     * @return vector<vector<int>>
     */
    Vec2int getVectorBoundaryGrid(Vec2int &grid) const;

    /**
     * transfer cv::Mat to vector<vector<int>>
     * @param src
     * @param m
     * @param n
     * @return
     */
    static vector<vector<int>> mat2Vector(Mat &src, int m, int n);

    /**
     * transfer vector<vector<int>> to cv::Mat
     * @param vec
     * @return
     */
    static Mat vector2Mat(vector<vector<int>> &vec);

    /**
     * Lidar Stimulation, fetch data from radar based on position currently
     * @param pos
     * @param Vec
     */
    void fetchLidarData(Pair &pos, Vec2int &Vec);

    /**
     * merge Intervals
     * @param newRange
     * @param intervals
     * @return
     */
    static vector<pair<double, double>> mergeInterval(const pair<double, double> &newRange,
                                               vector<pair<double, double>> &intervals);

    /**
     * check if the interval you input is included in intervals that you check
     * @param newRange
     * @param intervals
     * @return
     */
    static bool checkInterval(const pair<double, double> &newRange,
                                 vector<pair<double, double>> &intervals);

    /**
     * calculate degree range of a RECT
     * assuming radar located at point_0, target pos is point_d
     * @param point_0
     * @param point_d
     * @return
     */
    static pair<double, double> getThetaRange(Pair point_0, Pair point_d);

    /**
     * clear vector maps already scanned
     * cos 1 means visible and reachable, setting it as 1
     */
    void clearRadarMap();

    /**
     * launch radar
     */
    void launchRadar();

    /**
     * A star search while pos and grid is changing
     * @param pos
     * @param m_grid
     */
    void dynamicSearch(Pair &pos, Vec2int &m_grid);

    /**
     * trace path when A star is finished
     * @param cellDetails
     * @param pos
     */
    void dynamicTracePath(const vector<vector <cell>> &cellDetails, const Pair &pos);

    /**
     * check if there is Obstacle on Path calculated by A star
     * @param path
     * @return
     */
    PathVar ifPathObstacle(vector<Pair> &path);

    /**
     * show path using openCV
     */
    void showPath();
};


#endif //INC_5_FINAL_SOLUTION_H
