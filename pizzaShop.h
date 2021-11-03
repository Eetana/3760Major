#ifndef PIZZASHOP_H
#define PIZZASHOP_H

#include <ros/ros.h>


// Use std namespace for simplicity
using namespace std;

// The shop class is where the turtlebots belong to and manage them through implementing
// turtlebot3 class  (or pizzaBot class)
class Shop{
    public:
        void makePizza();

        void callBotsBack()

        // Due to the shop being open to the public, they can see if the pizza is ready
        // Similarly, the pizzaBot can see if the pizza is ready
        bool pizzaReady;
    private:
        // The shop is the only entity knowing where, when and what the pizzaBots are doing all the time
        // hence they should be the only people to keep track of them 
        lib<pizzaBot> droneList;


}