//
// Created by yzx on 8/20/20.
//

#ifndef MYSLAM_HELLO_H
#define MYSLAM_HELLO_H

#include <iostream>
#include <memory>
using namespace std;

class Hello{
public:
    typedef std::shared_ptr<Hello> Ptr;

    string name_;
    double age_;

public:
    Hello (){
        name_="Yang";
        age_=27;}

    Hello (string name,double age);
    void printHello();
    ~Hello(){
        cout<<"THis is being deleted"<<endl;
    }

};

#endif //MYSLAM_HELLO_H
