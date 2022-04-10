//
// Created by hazyparker on 2021/12/8.
//

#include "detectBlock.h"
#include <opencv2/opencv.hpp>
#include <utility>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace cv;

typedef vector<vector <int>> Vector2int;
typedef pair<int, int> Pair;

Vector2int getVectorMap(const char* filepath){
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
            img.at<uchar>(i, j) = int(m_map[i * cols + j]);
        }
    }

    // Mat to Vector2int
    vector<vector <int>> grid(m_mx, vector<int>(m_my));
    for (int i = 0; i < m_mx; i++){
        for (int j = 0; j < m_my; j++){
            // 1 is reachable
            // 0 is blocked
            if (int(img.at<uchar>(i, j)) == 0) grid[i][j] = 1;
            if (int(img.at<uchar>(i, j)) == 1) grid[i][j] = 0;
        }
    }

    return grid;
}

void showDetectBlocks(int i, int j){
    Vector2int grid = getVectorMap("../../maps/test_1.map");
    Pair center = {i, j};
    drawDetectedBlocks(grid, center);
}

void drawDetectedBlocks(Vector2int &grid, const Pair &center){
    /**
     * use RECT to stimulate a radar reflect area
     * radius means 1/2 l here, namely 2radius * 2radius RECT
     */
    int radius = 5;
    // FIXME： BUG here: radius less than 10
    int cor_i = center.first;
    int cor_j = center.second;

    // set minimum degree precision
    pair<double, double> precision;
    precision.first = atan2(radius - 0.5, 0.5);
    precision.second = atan2(radius - 0.5, -0.5);
    cout << "precision of degree is: " << (precision.second - precision.first) * 180 << endl;

    // set an array to record block sector, precision is 0.1 (360 / 3600 = 0.1)
    bool blockSector[3601];
    memset(blockSector, false, sizeof(blockSector));

    // test
    Mat img = imread("../images/test.jpg", IMREAD_COLOR);
    if (img.empty()){
        cout << "loading failed.." << endl;
        exit(100);
    }else{
        cout << "load successfully..." << endl;
    }
    Vec3b unseenColor = {0, 0, 255};
    Vec3b centerColor = {255, 0, 0};
    img.at<Vec3b>(center.first, center.second) = centerColor;

    //
    pair<double, double> range;
    for (int r = 1; r <= radius; r++){
        cout << "r=\t" << r << endl;
        for (int i = cor_i - r; i <= cor_i + r; i++){
            for (int j = cor_j - r; j <= cor_j + r; j++){
//                printf("(%d, %d) loop, ", i - cor_i, j - cor_j);
                // FIXME: a batter way for edge point's loop
                // only calculate edge points
                if (!isOnEdge({i, j}, center, r)){
//                    printf("(%d, %d) is not edge point\n", i, j);
                    continue;
                }

                if (grid[i][j] == 1){
//                    printf("(%d, %d) is not obstacle\n", i, j);
                    continue;
                }

                if (!isValid({i, j})){
                    printf("(%d, %d) is invalid\n", i, j);
                    return;
                }

                // get range
                range = getThetaRange(center, {i, j});
                double th_left = range.first;
                double th_right =range.second;

                if (range.second > 314 && range.first < 46 && grid[i][j] == 0){
                    for (int n = 0; n < int(th_left * 10); n++){
                        blockSector[n] = true;
                    }
                    for (int n = 3600 - int(th_right * 10) + 1; n <= 3600; n++){
                        blockSector[n] = true;
                    }
                    img.at<Vec3b>(i, j) = {0, 255, 0};
                    continue;
                }

                if (!blockSector[int(th_left * 10 + 1)] && !blockSector[int(th_right * 10 - 1)]){
//                    printf("(%d, %d) is unblocked\n", i, j);
                    for (int n = int(th_left * 10 + 1); n < int(th_right * 10 - 1); n++){
                        blockSector[n] = true;
                    }
                    img.at<Vec3b>(i, j) = {0, 255, 0};
                    continue;
                }if (blockSector[int(th_left * 10 + 1)] && blockSector[int(th_right * 10 - 1)]){
//                    printf("(%d, %d) is totally blocked\n", i, j);
                    // set as unseen
                    grid[i][j] = 1;
                    img.at<Vec3b>(i, j) = unseenColor;
                    continue;
                }if (!blockSector[int(th_left * 10 + 1)] || !blockSector[int(th_right * 10 - 1)]){
//                    printf("(%d, %d) is partly blocked\n", i, j);
                    for (int n = int(th_left * 10 + 1); n < int(th_right * 10 - 1); n++){
                        blockSector[n] = true;
                    }
                    img.at<Vec3b>(i, j) = {0, 255, 0};

                }
            }
        }
        cout << endl;
    }

    for (int i = 0; i < 3600; i ++){
        cout << i << ":\t" << blockSector[i] << endl;
    }
    cout << "precision of degree is: " << (precision.second - precision.first) * 180 << endl;
    namedWindow("test", WINDOW_NORMAL);
    imshow("test", img);
    waitKey(1000);

}

pair<double, double> getThetaRange(Pair point_0, Pair point_d){
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

bool isValid(Pair point){
    return (point.first >= 0) && (point.first < 100)
           && (point.second >= 0)
           && (point.second < 100);
}

bool isOnEdge(Pair point, Pair center, int r){
    bool seg1 = false;
    bool seg2 = false;
    if (point.first == center.first + r || point.first == center.first - r){
        seg1 = true;
    }
    if (point.second == center.second + r || point.second == center.second - r){
        seg2 = true;
    }

    return (seg1 || seg2);
}
