//
// Created by hazyparker on 2021/12/2.
//

/**
 * reference:
 * https://github.com/yhlleo/image2binarytest
 */


#include "ReadMap.h"
#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int readMap(const char* filename){
    int32_t m_mx, m_my, m_startX, m_startY, m_endX, m_endY;
    int8_t *m_map = nullptr;
    Mat img ,dst;

    FILE        *fp = nullptr;
    int         ret = -1;
    uint32_t    f_magic;

    // open file
    fp = fopen(filename, "rb");
    if( fp == nullptr ) {
        printf("ERR: failed to open file %s!\n", filename);
        return -1;
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

    ret = 0;

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
//            int gray_value = img.at<uchar>(i, j);
//
//            if (gray_value == 255){
//
//                if (i > 1 && i < cols - 2 && j > 1 && j < rows - 2 ){
//
//                    cout << "(i, j)=(" << i << ", " << j << "), " << "gray value nearby" << endl;
//                    for (int m = i-2; m <= i+2; m++){
//                        for (int n = j-2; n <= j+2; n++){
//                            cout << setw(4) << int(img.at<uchar>(m, n));
//
//                            if (int(img.at<uchar>(m, n)) != 255){
//                                img.at<uchar>(m, n) = 60;
//                            }
//
//                        }
//                        cout << endl;
//                    }
//
//                }
//            }
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

    // show image
    namedWindow("obstacle image", WINDOW_NORMAL);
    imshow("obstacle image", img);
    namedWindow("final image", WINDOW_NORMAL);
    imshow("final image", dst);
    waitKey(0);

    // write to local file
    imwrite("../images/readImage.jpg", dst);

    // show m_map array(int8_t to int)
    for (int i = 0; i < cols; i++){
        for (int j = 0; j < rows; j++){
            cout << int(m_map[i * cols + j]);
        }
        cout << endl;
    }

    return ret;
}

void ShowMap(){
    const char *filename = "../../maps/test_1.map";
    readMap(filename);
}
