#include <assert.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include "pointList.h"

using namespace std;

int main(){
    pointList test("happy");
    Point test2(2.,3.1,4.6);
    //test2.display(std::cout);
    test.display(std::cout);

    return 0;
}

