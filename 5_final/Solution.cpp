//
// Created by hazyparker on 2021/12/10.
//

#include "Solution.h"
#include "cmath"
#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <queue>
#include <tuple>
#include <vector>
#include <utility>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

typedef pair<int, int> Pair;
typedef vector<vector<int>> Vec2int;
typedef tuple<double, int, int> Tuple;
typedef tuple<bool, int, int, int> PathVar;

vector<Pair> OverallPath;

Vec2int Solution::getVectorRawGrid(const string& mapName) {
    string FILEPATH = "../../maps/" + mapName;
    const char *filepath = (char*)FILEPATH.data();
    // read map
    int32_t m_mx, m_my, m_startX, m_startY, m_endX, m_endY;
    int8_t *m_map;
    Mat img;

    FILE        *fp;
    uint32_t    f_magic;

    // open file
    fp = fopen(filepath, "rb");
    if( fp == nullptr ) {
        printf("ERR: failed to open file %s!\n", filepath);
        exit(100);
    }

    // check file magic number
    fread(&f_magic, sizeof(uint32_t), 1, fp);
    if( f_magic != 0x15432345 ) {
        printf("ERR: input file format is not correct! %s\n", filepath);
        goto MAP_LOAD_RET;
    }

    // read map nx, ny
    fread(&m_mx, sizeof(int32_t), 1, fp);
    fread(&m_my, sizeof(int32_t), 1, fp);
    ROW = m_mx;
    COL = m_my;

    // read map
    m_map = new int8_t[m_mx * m_my];
    fread(m_map, sizeof(int8_t), m_mx * m_my, fp);

    // read start,end point
    fread(&m_startX, sizeof(int32_t), 1, fp);
    fread(&m_startY, sizeof(int32_t), 1, fp);
    fread(&m_endX, sizeof(int32_t), 1, fp);
    fread(&m_endY, sizeof(int32_t), 1, fp);
    StartPoint = {m_startX, m_startY};
    EndPoint = {m_endX, m_endY};

    MAP_LOAD_RET:
    fclose(fp);

    int cols = m_mx;
    // Mat to Vector2int
    for (int i = 0; i < m_mx; i++){
        for (int j = 0; j < m_my; j++){
            // 1 is reachable
            // 0 is blocked
            if (int(m_map[i * cols + j]) == 0){
                map_vector[i][j] = 1;
            }
            if (int(m_map[i * cols + j]) == 1){
                map_vector[i][j] = 0;
            }
        }
    }

    return map_vector;
}

Pair Solution::cor2ArrayPos(Pair corPos) {
    Pair ArrayPos;
    ArrayPos.first = corPos.second - 1;
    ArrayPos.second = corPos.first - 1;
    return ArrayPos;
}

void Solution::setStartPoint(int x, int y) {
    Pair corPos;
    corPos.first = x;
    corPos.second = y;

    StartPoint = cor2ArrayPos(corPos);
}

void Solution::setEndPoint(int x, int y) {
    Pair corPos;
    corPos.first = x;
    corPos.second = y;

    EndPoint = cor2ArrayPos(corPos);
}

bool Solution::isValid(const vector<vector<int>> &grid, const Pair &point) const {
    // Returns true if row number and column number is in range
    if (ROW > 0 && COL > 0)
        return (point.first >= 0) && (point.first < ROW)
               && (point.second >= 0)
               && (point.second < COL);

    else return false;
}

bool Solution::isUnBlocked(const vector<vector<int>> &grid, const Pair &point) const {
    // Returns true if the cell is not blocked else false
    return isValid(grid, point)
           && grid[point.first][point.second] == 1;
}

bool Solution::isDestination(const Pair &position) {
    return position == EndPoint;
}

double Solution::calculateHValue(const Pair &src, const Pair &dest) {
    // h is estimated with the two points distance formula
    return sqrt(pow((src.first - dest.first), 2.0)
                + pow((src.second - dest.second), 2.0));
}

Mat Solution::vector2ObstacleMap(Vec2int &grid) const {
    Mat img, dst;
    img = Mat(ROW, COL, CV_8U, Scalar::all(255));

    // convert m_map to int, sending to img
    for (int i = 0; i < COL; i++){
        for (int j = 0; j < ROW; j++){
            img.at<uchar>(i, j) = grid[i][j] * 255;
        }
    }

    // erode, showing the edge
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(img, dst, element);

    // uplift the gray scale of edges
    for (int i = 0; i < COL; i++){
        for (int j = 0; j < ROW; j++){
            if(int(dst.at<uchar>(i, j)) == 0)
                dst.at<uchar>(i, j) = 200;
        }
    }

    // blend two images, using lower gray scale
    // consider using addWeight()
    for (int i = 0; i < COL; i++){
        for (int j = 0; j < ROW; j++){
            if(int(dst.at<uchar>(i, j)) == 200 && int(img.at<uchar>(i, j)) == 0)
                dst.at<uchar>(i, j) = 0;
        }
    }

    return dst;
}

Vec2int Solution::getVectorBoundaryGrid(Vec2int &grid) const {
    Mat img, dst;
    vector<vector <int>> boundaryGrid(ROW, vector<int>(COL));

    img = vector2Mat(grid);
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(img, dst, element);

    boundaryGrid = mat2Vector(dst, ROW, COL);
    return boundaryGrid;
}

vector<vector <int>> Solution::mat2Vector(Mat &src, int m, int n) {
    vector<vector <int>> transferredVector(m, vector<int>(n));
    vector<int> flatVector = src.reshape(1,1);
    // FIXME: how to transfer Mat to vector<vector>
//    transferredVector = flatVector.resize(m, 0);
    for (int i = 0; i < m; i++){
        for (int j = 0; j < m; j++){
            transferredVector[i][j] = int(src.at<uchar>(i, j));
        }
    }
    return transferredVector;
}

Mat Solution::vector2Mat(vector<vector <int>> &vec) {
    Mat dst;
    dst = Mat(100, 100, CV_8U, Scalar::all(0));
    for (int i = 0; i < 100; i++){
        for (int j = 0; j < 100; j++){
            dst.at<uchar>(i, j) = vec[i][j];
        }
    }
    return dst;
}

void Solution::fetchLidarData(Pair &pos, Vec2int &Vec) {
    // set radius
    int radius = 40;
    int pos_i = pos.first;
    int pos_j = pos.second;

    // open Lidar
    pair<double, double> range;
    vector<pair<double, double>> degreeInterval;
    degreeInterval.emplace_back(0.001, 0.002);
    int i, j;
    for (int r = 1; r <= radius; r++){
        for (i = pos_i - r; i <= pos_i + r; i++){
            for (j = pos_j - r; j <= pos_j +r; j++){
                if (!isValid(map_vector, {i, j})) continue;
                if (abs(i - pos_i) + abs(j - pos_j) != r || map_vector[i][j] == 1) continue;

                range = getThetaRange(pos, {i, j});
                if (range.second > 314 && range.first < 46 && map_vector[i][j] == 0){
                    degreeInterval = mergeInterval({0.0001, range.first}, degreeInterval);
                    degreeInterval = mergeInterval({range.second, 359.9999}, degreeInterval);
                    Vec[i][j] = 0;
                    continue;
                }
                if (checkInterval(range, degreeInterval)){
                    // if totally blocked by other obstacles, UAV was unaware of this obstacle
                    // have to consider another situation, that this [i][j] had been checked before
                    // and set as 0 which means unreachable, we can't make it invisible again in radar_map
                    // however, in image shown as to having used radar, it should be erased again
                    // that step is not necessary(for now)
                    // if (radar_map[i][j] == 0) radar_map[i][j] = 0;
                    // if (radar_map[i][j] == 1) radar_map[i][j] = 1;

                }else{
                    Vec[i][j] = 0;
                }
                degreeInterval = mergeInterval(range, degreeInterval);
            }
        }
    }

}

vector<pair<double, double>> Solution::mergeInterval(const pair<double, double> &newRange,
                                           vector<pair<double, double>> &intervals) {
    // https://leetcode-cn.com/problems/merge-intervals/solution/he-bing-qu-jian-by-leetcode-solution/

    intervals.emplace_back(newRange);

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

bool Solution::checkInterval(const pair<double, double> &newRange,
                   vector<pair<double, double>> &intervals) {
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

pair<double, double> Solution::getThetaRange(Pair point_0, Pair point_d){
    // point_0 center
    // point_d para

    double PI = 3.1415926;

    pair<double, double> range;

    int i_0 = point_0.first;
    int j_0 = point_0.second;
    int i_d = point_d.first;
    int j_d = point_d.second;

    // save theta
    double theta[4] = {0, 0, 0, 0};
    // range change from (-PI, PI) to (0, 2PI)
    // FIXME: special definition around -PI
    theta[0] = atan2(i_d + 0.5 - i_0, j_d + 0.5 - j_0) * 180 / PI + 180;
    theta[1] = atan2(i_d + 0.5 - i_0, j_d - 0.5 - j_0) * 180 / PI + 180;
    theta[2] = atan2(i_d - 0.5 - i_0, j_d + 0.5 - j_0) * 180 / PI + 180;
    theta[3] = atan2(i_d - 0.5 - i_0, j_d - 0.5 - j_0) * 180 / PI + 180;
    sort(theta, theta + 4);
    // if theta[3] >= 315, i_0 == i_d (for sure)

    // send to pair range
    range.second = theta[3];
    range.first = theta[0];

    if (theta[3] > 340 && theta[0] < 19){
        range.second = theta[2];
        range.first = theta[1];
    }

    return range;
}

void Solution::clearRadarMap() {
    cout << "map cleared" << endl;
    for (int i = 0; i < ROW; i++){
        for (int j = 0; j < COL; j++){
            radar_vector[i][j] = 1;
            check_vector[i][j] = 1;
            show_vector[i][j] = 1;
        }
    }
}

void Solution::launchRadar() {
    // clear all obstacles, forming a new map
    clearRadarMap();
    // initialize start point as StartPoint
    Pair pos = StartPoint;
    fetchLidarData(pos, radar_vector);

    int number = 0;
    cout << "into loop launch.." << endl;

    while(pos != EndPoint){
        // clear check_vector
        for (int i = 0; i < ROW; i++){
            for (int j = 0; j < COL; j++){
                check_vector[i][j] = 1;
            }
        }

        cout << "loop:" << number << endl;
        number = number + 1;

        printf("(%d, %d) pos\n", pos.first, pos.second);

        // store path in stack DynamicStack
        dynamicSearch(pos, radar_vector);

        // test if obstacle on path
        vector<Pair> path;
        while(!DynamicPath.empty()){
            Pair p = DynamicPath.top();
            DynamicPath.pop();
            path.emplace_back(p);
        }
        cout << "path data read in..." << endl;
        for (auto & i : path){
            cout << "data in path:" << i.first << ", " << i.second << endl;
        }

        PathVar message = ifPathObstacle(path);

        cout << get<0>(message) << endl;
        if (get<0>(message)){
            // update pos to last one
            pos = {get<1>(message), get<2>(message)};
            int num = get<3>(message);
            for (int i = 0; i <= num; i++){
                fetchLidarData(path[i], radar_vector);
                OverallPath.emplace_back(path[i]);

                cout << path[i].first << ", " << path[i].second << endl;
            }
        }if (!get<0>(message)){
            for (auto & i : path){
                OverallPath.push_back(i);
            }
            pos = EndPoint;
            cout << "end founded..." << endl;
        }
    }

}

void Solution::dynamicSearch(Pair &pos, Vec2int &m_grid) {
    // set begin time
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    Vec2int grid(ROW, vector<int>(COL));
    grid = getVectorBoundaryGrid(m_grid);

    // If the source is out of range
    if (!isValid(grid, StartPoint)) {
        printf("Source is invalid\n");
        return;
    }

    // If the destination is out of range
    if (!isValid(grid, EndPoint)) {
        printf("Destination is invalid\n");
        return;
    }

    // Either the source or the destination is blocked
    if (!isUnBlocked(grid, StartPoint)
        || !isUnBlocked(grid, EndPoint)) {
        printf("Source or the destination is blocked\n");
        return;
    }

    // If the destination cell is the same as source cell
    if (isDestination(StartPoint)) {
        printf("We are already at the destination\n");
        return;
    }

    // describe detail of each cell
    // using struct defined
    vector<vector <cell>> cellDetails(ROW, vector<cell>(COL));

    int i, j;
    // Initialising the parameters of the starting node
    i = pos.first, j = pos.second;
    cellDetails[i][j].cost_f = 0.0;
    cellDetails[i][j].cost_g = 0.0;
    cellDetails[i][j].cost_h = 0.0;
    cellDetails[i][j].nodeParent = {i, j};

    // Create a closed list and initialise it to false
    bool closedList[ROW][COL];
    memset(closedList, false, sizeof(closedList));

    /**
     * Create an open list having information as <f, <i, j>>
     * where f = g + h,
     * and i, j are the row and column index of that cell
     * Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
     * This open list is implemented as a set of tuple
     */
    priority_queue<Tuple, vector<Tuple>, greater<> > openList;
    // Type: Tuple
    // Container: vector
    // Functional: greater, least f will be on top of the queue

    openList.emplace(0.0, i, j);
    while(!openList.empty()){
        const Tuple& p = openList.top(); // first time, it will be start point
        // Add this vertex to the closed list
        i = get<1>(p); // second element of tuple
        j = get<2>(p); // third element of tuple

        // remove this vertex from open list and send it to close list
        openList.pop();
        closedList[i][j] = true;

        for (int add_x = -1; add_x <= 1; add_x++) {
            for (int add_y = -1; add_y <= 1; add_y++) {

                Pair neighbour(i + add_x, j + add_y);

                if (!isValid(grid, neighbour)){
                    return;
                }
                if (isDestination(neighbour)){
                    // set parent of destination cell
                    cellDetails[neighbour.first][neighbour.second].nodeParent = {i, j};
                    // print path
                    cout << "destination is found!" << endl;
                    dynamicTracePath(cellDetails, pos);
                    // set end time and show
                    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
                    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
                    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
                    // end
                    return;
                }
                else if (!closedList[neighbour.first][neighbour.second]
                         && isUnBlocked(grid, neighbour)){

                    // update weight values
                    double f_updated, g_updated, h_updated;
                    g_updated = cellDetails[i][j].cost_g + sqrt(pow(add_x, 2.0) + pow(add_y, 2.0));
                    h_updated = calculateHValue(neighbour, EndPoint);
                    f_updated = g_updated + h_updated;

                    // send less f_updated to open list
                    double temp_cost_f = cellDetails[neighbour.first][neighbour.second].cost_f;
                    if (f_updated < temp_cost_f || temp_cost_f == -1){
                        // -1 referring that this node doesn't belong to open list, shall be added in
                        // f_updated is less, shall be added in open list
                        // at first I was concerning about the StartPoint, then I just realized that
                        // StartPoint has been thrown into close list
                        openList.emplace(f_updated, neighbour.first, neighbour.second);

                        // update cost f, g, h and parent node of cellDetails
                        cellDetails[neighbour.first][neighbour.second].cost_f = f_updated;
                        cellDetails[neighbour.first][neighbour.second].cost_g = g_updated;
                        cellDetails[neighbour.first][neighbour.second].cost_h = h_updated;
                        // set parent of successors(not in Close list and is unblocked)
                        cellDetails[i + add_x][j + add_y].nodeParent = {i, j};
                        /**
                         * why updating parent node must exist here?
                         * if you actually put this update under definition of neighbour, some path might be confusing
                         * set updating here can assure that each node parent heading for gradient lifting
                         * namely f in decline
                         */
                    }

                }

            }

        }

    }


}

void Solution::dynamicTracePath(const vector<vector <cell>> &cellDetails, const Pair &pos) {
    cout << "trace started..." << endl;
    int i = EndPoint.first;
    int j = EndPoint.second;
    Pair temp_node;

    while(cellDetails[i][j].nodeParent != pos){
        DynamicPath.push({i, j});
        temp_node = cellDetails[i][j].nodeParent;
        // i = cellDetails[i][j].nodeParent.first is illegal!
        i = temp_node.first;
        j = temp_node.second;
    }
    DynamicPath.push({i, j});
    // push StartPoint into Stack
    DynamicPath.push(pos);
    cout << "trace finished" << endl;
}

PathVar Solution::ifPathObstacle(vector<Pair> &path){
    PathVar message;
    bool seg;

    // FIXME: check_map not best

    for (int n = 0; n < path.size(); n++){
        int i = path[n].first;
        int j = path[n].second;

        Pair pos = {i, j};
        fetchLidarData(pos, check_vector);

        for (auto & k : path){
            if (check_vector[k.first][k.second] == 0){
                seg = true;
                get<1>(message) = i;
                get<2>(message) = j;
                get<3>(message) = n;
                get<0>(message) = seg;
                printf("(%d, %d) is on obstacle\n", k.first, k.second);
                return message;
            }else{
                seg = false;
                get<0>(message) = seg;
            }
        }

    }

    return message;
}

void Solution::showPath() {
    for (int i = 0; i < ROW; i++){
        for (int j = 0; j < COL; j++){
            show_vector[i][j] = 1;
        }
    }

    Mat img;
    img = Mat(100, 100, CV_8UC3, Scalar::all(255));
    Vec2int temp_map (ROW, vector<int>(COL));
    for (auto & i : OverallPath){
        fetchLidarData(i, show_vector);
        temp_map = getVectorBoundaryGrid(show_vector);

        for (int x = 0; x < ROW; x++){
            for (int y = 0; y < COL; y++){
                if (show_vector[x][y] == 0) img.at<Vec3b>(x, y) = {0, 0, 0};
                if (show_vector[x][y] == 1 && temp_map[x][y] == 0)
                    img.at<Vec3b>(x, y) = {0, 128, 128};
            }
        }
        img.at<Vec3b>(i.first, i.second) = {0, 255, 0};
        img.at<Vec3b>(StartPoint.first, StartPoint.second) = {255, 0, 0};
        img.at<Vec3b>(EndPoint.first, EndPoint.second) = {0, 0, 255};
        namedWindow("dst", WINDOW_NORMAL);
        imshow("dst", img);

        waitKey(100);
    }

}

