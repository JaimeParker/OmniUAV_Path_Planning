//
// Created by hazyparker on 2021/12/10.
//

#include "testRadar.h"
#include "AStarSearch.h"
#include <utility>

void showRadarGet(){
    AStar a;
    a.setMapName("test_1.map");
    pair<int, int> center = {62, 53};
//    for (int i = 0; i < 20; i++){
//        center.first = center.first - i;
//        a.openLidar(center);
//    }
    a.openLidar(center);
}
