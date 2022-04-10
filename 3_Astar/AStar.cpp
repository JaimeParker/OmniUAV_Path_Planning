//
// Created by hazyparker on 2021/12/4.
//

#include "AStar.h"
#include "readMap.h"
#include "cmath"
#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <queue>
#include <set>
#include <tuple>
#include <vector>
#include <utility>
#include <opencv2/opencv.hpp>

using namespace std;

typedef pair<int, int> Pair;
typedef tuple<double, int, int> Tuple;

AStar::AStar() {
    mapPath = "../../maps/test_1.map";
    binaryObstacleMap = getMap();
    ROW = 100;
    COL = 100;
}

Pair AStar::cor2ArrayPos(Pair corPos) {
    Pair ArrayPos;
    ArrayPos.first = corPos.second - 1;
    ArrayPos.second = corPos.first - 1;
    return ArrayPos;
}

Pair AStar::array2CorPos(Pair arrayPos) {
    Pair corPos;
    corPos.first = arrayPos.second + 1;
    corPos.second = arrayPos.first + 1;
    return corPos;
}

void AStar::setStartPoint(int x, int y) {
    Pair corPos;
    corPos.first = x;
    corPos.second = y;

    StartPoint = cor2ArrayPos(corPos);
}

void AStar::setEndPoint(int x, int y) {
    Pair corPos;
    corPos.first = x;
    corPos.second = y;

    EndPoint = cor2ArrayPos(corPos);
}

bool AStar::isValid(const vector<vector<int>> &grid, const Pair &point) const {
    // Returns true if row number and column number is in range
    if (ROW > 0 && COL > 0)
        return (point.first >= 0) && (point.first < ROW)
               && (point.second >= 0)
               && (point.second < COL);

    else return false;
}

bool AStar::isUnBlocked(const vector<vector<int>> &grid, const Pair &point) const {
    // Returns true if the cell is not blocked else false
    return isValid(grid, point)
           && grid[point.first][point.second] == 1;
}

bool AStar::isDestination(const Pair &position) {
    return position == EndPoint;
}

double AStar::calculateHValue(const Pair &src, const Pair &dest) {
    // h is estimated with the two points distance formula
    return sqrt(pow((src.first - dest.first), 2.0)
                + pow((src.second - dest.second), 2.0));
}

void AStar::aStarSearch(){
    binaryObstacleMap = getBinaryEdgeMap();
    vector<vector <int>> grid;
    grid = OPS_CV::mat2Vector(binaryObstacleMap, ROW, COL);
    // set begin time
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

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
    i = StartPoint.first, j = StartPoint.second;
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

        printf("parent node now:(%d, %d)\n", i, j);

        for (int add_x = -1; add_x <= 1; add_x++) {
            for (int add_y = -1; add_y <= 1; add_y++) {

                printf("added vector now:(%d, %d)\n", i + add_x, j + add_y);

                Pair neighbour(i + add_x, j + add_y);

                if (!isValid(grid, neighbour)){
                    cout << neighbour.first << ", " << neighbour.second << "is invalid";
                    return;
                }
                if (isDestination(neighbour)){
                    // set parent of destination cell
                    cellDetails[neighbour.first][neighbour.second].nodeParent = {i, j};
                    // print path
                    cout << "destination is found!" << endl;
                    tracePath(cellDetails);
                    // set end time and show
                    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
                    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
                    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
                    // end
                    return;
                }
                else if (!closedList[neighbour.first][neighbour.second]
                         && isUnBlocked(grid, neighbour)){

                    printf("(%d, %d) will be calculated\n", neighbour.first, neighbour.second);

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

                        printf("(%d, %d) added to open list\n", neighbour.first, neighbour.second);

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

void AStar::tracePath(const vector<vector<cell>> &cellDetails) {
    /**
     * there must be a circle
     * until cellDetails[][].nodeParent = StartPoint
     * we shall start from EndPoint, getting its parent, test the condition above
     * then go into next loop
     */
    int i = EndPoint.first;
    int j = EndPoint.second;
    Pair temp_node;
    while(cellDetails[i][j].nodeParent != StartPoint){
        Path.push({i, j});
        temp_node = cellDetails[i][j].nodeParent;
        // i = cellDetails[i][j].nodeParent.first is illegal!
        i = temp_node.first;
        j = temp_node.second;
    }
    Path.push({i, j});
    // push StartPoint into Stack
    Path.push(StartPoint);
}

void AStar::drawPath() {
    ObstacleMap = readMap(mapPath);
    Mat pathImg;
    ObstacleMap.copyTo(pathImg);
    cvtColor(pathImg, pathImg, COLOR_GRAY2BGR);

    // set path color, BGR order
    Vec3b pathColor;
    pathColor[0] = 0;
    pathColor[1] = 255;
    pathColor[2] = 0;

    Vec3b startColor = {0, 0, 255};

    Vec3b endColor = {255, 0, 0};

    // draw start and end point
    pathImg.at<Vec3b>(StartPoint.first, StartPoint.second) = startColor;
    pathImg.at<Vec3b>(EndPoint.first, EndPoint.second) = endColor;

    // form window
    namedWindow("Obstacle map", WINDOW_NORMAL);
    namedWindow("path on map", WINDOW_NORMAL);

    int i, j;
    Pair p_cor;
    while (!Path.empty()) {
        Pair p = Path.top();
        Path.pop();
        i = p.first;
        j = p.second;
        p_cor = array2CorPos(p);
        printf("(%d, %d)->\n", p_cor.first, p_cor.second);
        pathImg.at<Vec3b>(i, j) = pathColor;
        pathImg.at<Vec3b>(StartPoint.first, StartPoint.second) = startColor;
        pathImg.at<Vec3b>(EndPoint.first, EndPoint.second) = endColor;
        imshow("path on map", pathImg);
        waitKey(100);
    }

    imshow("Obstacle map", ObstacleMap);
    waitKey(0);

    imwrite("../images/path1.jpg", pathImg);
}

Mat AStar::readMap(const string &filename) {
    int32_t m_mx, m_my, m_startX, m_startY, m_endX, m_endY;
    int8_t *m_map;
    Mat img ,dst;

    FILE        *fp;
    uint32_t    f_magic;

    // open file
    const char *filepath = (char*)filename.data();
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

    // read map
    m_map = new int8_t[m_mx * m_my];
    fread(m_map, sizeof(int8_t), m_mx * m_my, fp);

    // read start,end point
    fread(&m_startX, sizeof(int32_t), 1, fp);
    fread(&m_startY, sizeof(int32_t), 1, fp);
    fread(&m_endX, sizeof(int32_t), 1, fp);
    fread(&m_endY, sizeof(int32_t), 1, fp);

    MAP_LOAD_RET:
    fclose(fp);

    int cols = m_mx;
    int rows = m_my;
    img = Mat(rows, cols, CV_8U, Scalar::all(255));

    // convert m_map to int, sending to img
    for (int i = 0; i < cols; i++){
        for (int j = 0; j < rows; j++){
            img.at<uchar>(i, j) = int(m_map[i * cols + j]) * 255;
        }
    }

    // LUT
    uchar lutData[256];
    for (int i = 0; i < 256; ++i){
        lutData[i] = 255 - i;
    }
    Mat lutTable(1,256, CV_8U);
    uchar *p = lutTable.ptr();
    for (int i = 0; i < 256; ++i){
        p[i] = lutData[i];
    }

    LUT(img, lutTable, img);

    // erode, showing the edge
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(img, dst, element);

    // uplift the gray scale of edges
    for (int i = 0; i < cols; i++){
        for (int j = 0; j < rows; j++){
            if(int(dst.at<uchar>(i, j)) == 0)
                dst.at<uchar>(i, j) = 200;
        }
    }

    // blend two images, using lower gray scale
    // consider using addWeight()
    for (int i = 0; i < cols; i++){
        for (int j = 0; j < rows; j++){
            if(int(dst.at<uchar>(i, j)) == 200 && int(img.at<uchar>(i, j)) == 0)
                dst.at<uchar>(i, j) = 0;
        }
    }

    dst.copyTo(ObstacleMap);
    return ObstacleMap;
}

void AStar::setMapName(const string& filename) {
    mapPath = "../../maps/" + filename;
}

Mat AStar::getBinaryEdgeMap() {
    ObstacleMap = readMap(mapPath);
    Mat src = ObstacleMap;
    /**
     * if src.at = 1, it's free to go
     * if src.at = 0, it's blocked
     */
    for (int i = 0; i < 100; i++){
        for (int j = 0; j < 100; j++){
            if(src.at<uchar>(i, j) != 255) src.at<uchar>(i, j) = 0;
            if(src.at<uchar>(i, j) == 255) src.at<uchar>(i, j) = 1;
        }
    }
    return src;
}
