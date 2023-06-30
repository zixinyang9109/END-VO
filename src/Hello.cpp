//
// Created by yzx on 8/20/20.
//

#include "myslam//Hello.h"

Hello::Hello (string name,double age):name_(name),age_(age) {
}


void Hello::printHello() {
    cout<<"Hello "<<name_<<"you are "<<age_<<" years old"<<endl;
}

#if 0
int main()
{
    Hello Yang("Zixin YANG",30);
    Yang.printHello();
    Hello::Ptr yang(new Hello);
    yang->printHello();

    //Yang.name="Zixin";
    //Yang.age=24;

    //cout<<Yang.name<<"is "<<Yang.age<<Yang.age<<endl;
}
#endif

