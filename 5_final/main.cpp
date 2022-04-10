#include "Solution.h"

using namespace std;
using namespace cv;

int main() {
vector<vector<int>> grid;
    Solution solve;
    solve.getVectorRawGrid("test_1.map");
    solve.setStartPoint(40, 10);
    solve.setEndPoint(40, 90);
    solve.launchRadar();
    solve.showPath();

    return 0;
}
