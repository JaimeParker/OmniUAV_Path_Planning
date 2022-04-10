//
// Created by hazyparker on 2021/12/9.
//

#ifndef INC_5_FINAL_READMAP_H
#define INC_5_FINAL_READMAP_H
#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

class OPS_CV {
public:
    /**
     * transfer 2D Mat to 2D vector(STL, C++)
     * @param src A Mat in openCV
     * @param m m = rows
     * @param n n = cols
     * @return vector<vector<int>>
     */
    static vector<vector <int>> mat2Vector(Mat &src, int m, int n);

    /**
     * transfer 2D vector(STL, C++) to 2D Mat in OpenCV
     * @param vec 2D vector
     * @param channels gray scale or RGB, 1 or 3
     * @param rows no need for cols
     * @return Mat
     */
    template<typename Tp>
    static Mat vector2Mat(vector<vector <Tp>> &vec, int channels, int rows);




};

Mat readMap(const char* filename);
Mat getMap();

Mat edgeMap(Mat &src);


#endif //INC_5_FINAL_READMAP_H
