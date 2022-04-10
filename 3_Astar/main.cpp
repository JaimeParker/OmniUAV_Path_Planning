#include "AStar.h"

using namespace std;

// Driver program to test above function
int main()
{
    AStar PathPlanning;
    PathPlanning.setMapName("test_1.map");
    PathPlanning.setStartPoint(40, 10);
    // 40, 26
    PathPlanning.setEndPoint(40, 90);
    // 40， 83
    PathPlanning.aStarSearch();
    PathPlanning.drawPath();

    return 0;
}



