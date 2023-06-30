//
// Created by yzx on 8/9/21.
//

#include <iostream>
#include <vector>
#include "myslam/Hello.h"
#include "myslam/common_include.h"

int main() {
    std::cout << "Hello, World!" << std::endl;

    Hello Yang("Zixin YANG",30);
    Yang.printHello();
    Hello::Ptr yang(new Hello);
    yang->printHello();
    std::vector<cv::Point2f> mvbP;
    mvbP.resize(10);


    return 0;
}