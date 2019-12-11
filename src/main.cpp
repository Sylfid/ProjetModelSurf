#include <assert.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include "pointList.h"
#include "point.h"

using namespace std;

int main(){
    //pointList test("happy");
    Point test2(2.,3.1,4.6);
    Point test1(3, 4, 5);
    std::cout << test2.getDistanceTo(test1);
    //test2.display(std::cout);
    //test.display(std::cout);

    return 0;
}
