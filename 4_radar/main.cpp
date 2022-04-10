#include "detectBlock.h"
#include "degreeVectorMethod.h"

#include <vector>
#include <utility>
#include <iostream>

using namespace std;

int main() {
//    for (int i = 62; i > 42; i--){
//        showDetectBlocks(i, 53);
//    }

//    showDetectBlocks(62, 53);
    for (int i = 10; i < 70; i++){
        openLidar({i, 40});
    }
    // FIXME: small issues


    return 0;
}
