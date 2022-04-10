//
// Created by hazyparker on 2021/12/4.
//

#ifndef INC_3_ASTAR_ASTAR_H
#define INC_3_ASTAR_ASTAR_H

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

class AStar {
private:
    Mat binaryObstacleMap;
    Mat ObstacleMap;
    int ROW;
    int COL;
    Pair StartPoint;
    Pair EndPoint;
    stack<Pair> Path;
    string mapPath;

public:
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
     * constructor function
     */
    AStar();

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


    void aStarSearch();

    /**
     * transfer coordinate position to (i, j) display in array which starting at [0]
     * @param corPos pair(STL, C++) coordinate position of a point
     * @return array position
     */
    static Pair cor2ArrayPos(Pair corPos);

    /**
     * transfer array position to coordinate position
     * @param arrayPos
     * @return corPos
     */
    static Pair array2CorPos(Pair arrayPos);

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
     * trace path of the shortest path
     * @param cellDetails
     */
    void tracePath(const vector<vector <cell>> &cellDetails);

    // draw path
    void drawPath();

    Mat readMap(const string &filename);

    void setMapName(const string& filename);

    Mat getBinaryEdgeMap();
};


#endif //INC_3_ASTAR_ASTAR_H
