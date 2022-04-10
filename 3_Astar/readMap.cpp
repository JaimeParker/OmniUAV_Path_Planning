//
// Created by hazyparker on 2021/12/4.
//

#include "readMap.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

Mat readMap(const char* filename){
    int32_t m_mx, m_my, m_startX, m_startY, m_endX, m_endY;
    int8_t *m_map = nullptr;
    Mat img ,dst;

    FILE        *fp = nullptr;
    uint32_t    f_magic;

    // open file
    fp = fopen(filename, "rb");
    if( fp == nullptr ) {
        printf("ERR: failed to open file %s!\n", filename);
        exit(100);
    }

    // check file magic number
    fread(&f_magic, sizeof(uint32_t), 1, fp);
    if( f_magic != 0x15432345 ) {
        printf("ERR: input file format is not correct! %s\n", filename);
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

    cout << "type of m_map:" <<typeid(m_map).name() << endl; // get typeof()

    MAP_LOAD_RET:
    fclose(fp);
    cout << "file closed" << endl; // no RE read after file closed

    int cols = m_mx;
    int rows = m_my;
    cout << "rows:" << rows << ", cols:" << cols << endl;
    cout << "start pos:(" << m_startX << ", " << m_startY << ")" << endl;
    cout << "end pos:(" << m_endX << ", " << m_endY << ")" << endl;
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
    return dst;
}

Mat getMap(){
    const char *filename = "../../maps/test_1.map";
    Mat img =  readMap(filename);
    Mat dst = edgeMap(img);
    return dst;
}

Mat edgeMap(Mat &src){
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

Mat gray2RGB(Mat &src){
    Mat dst;
    cvtColor(src, dst, COLOR_GRAY2BGR);
    return dst;
}

vector<vector <int>> OPS_CV::mat2Vector(Mat &src, int m, int n) {
    vector<vector <int>> transferredVector(m, vector<int>(n));
    vector<int> flatVector= src.reshape(1,1);
    // FIXME: how to transfer Mat to vector<vector>
//    transferredVector = flatVector.resize(m, 0);
    for (int i = 0; i < m; i++){
        for (int j = 0; j < m; j++){
            transferredVector[i][j] = int(src.at<uchar>(i, j));
        }
    }
    return transferredVector;
}

template<typename Tp>
Mat OPS_CV::vector2Mat(vector<vector <Tp>> &vec, int channels, int rows) {
    Mat mat = Mat(vec);
    Mat dst =  mat.reshape(channels, rows).clone();
    return dst;
}


