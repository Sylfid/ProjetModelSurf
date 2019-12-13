#include <assert.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include "vector3dList.h"
#include "vector3d.h"

using namespace std;

int main(){
    //pointList test("happy");
    Vector3d test2(2.,3.1,4.6);
    Vector3d test1(3, 4, 5);
    std::cout << test2.getDistanceTo(test1);
    //test2.display(std::cout);
    //test.display(std::cout);

    return 0;
}
