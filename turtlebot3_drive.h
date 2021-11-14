#ifndef TURTLEBOT3_DRIVE_H_
#define TURTLEBOT3_DRIVE_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/tfMessage.h>
#include <tf2_msgs/TFMessage.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <array>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#define DEG2RAD (M_PI / 180.0)    // transfer degree to radius
#define RAD2DEG (180.0 / M_PI)    // transfer radius to degree     

// There are the angles which would be used to check the barrier, because the laser will detecte range around 360 degree.
// We don't need to check all of its.  
#define CENTER    0
#define LEFT_15   1
#define LEFT_30   2
#define LEFT_45  3
#define LEFT_60  4
#define LEFT_75  5
#define LEFT_90  6
#define FOR_BACK 7
#define RIGHT_270  8
#define RIGHT_285  9
#define RIGHT_300  10
#define RIGHT_315  11
#define RIGHT_330  12
#define RIGHT_345  13

// Define the robot moving velocity and the cornering speed which call angular velocity
#define LINEAR_VELOCITY  0.2   
#define ANGULAR_VELOCITY 1.3
#define LOWER_ANGULAR_VELOCITY 0.5   

// define the direction using there are 4 different instructions
#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1     // ask robot move forward
#define TB3_RIGHT_TURN    2     // ask robot turn right
#define TB3_LEFT_TURN     3     // ask robot turn left

class CLaser
{
 public:
   void initLaser(ros::NodeHandle *nh_);
   void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
   enum movementDirection{right_turn, straight, left_turn, left_collision_turn};
   movementDirection checkMovementDirection(void);
 private:
   ros::Subscriber laser_scan_sub_;
   double check_forward_dist_;
   double check_side_dist_;
   double scan_data_[13] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

class COdom
{
 public:
   void initOdom(ros::NodeHandle *nh_);
   void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
   double tb3_pose_;
   double prev_tb3_pose_;
 private:
   ros::Subscriber odom_sub_;
};

class CCamera
{
 public:
  void initCamera(ros::NodeHandle *nh_);
  void tagDetectMsgCallBack(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
  enum addressDetected{no_address, pizza_shop, not_pizza_shop};
  addressDetected checkAddress(int pizza_shop_address);
  addressDetected checkIfPizzaShop(void);

  bool tag_detected=false;
  int tag_id;

 private:
  ros::Subscriber tag_detections_sub_;
  float x_tag_pose;
  float y_tag_pose;
  float z_tag_pose;
};

class CPizzaDeliverer;

class CPizzaOrder
{
  public:
    CPizzaOrder createOrder(void);
    int no_pizza_in_order;
    int pizza_add;
    bool delivered_status;
};

class CPizzaStore
{
  public:
    void prepareOrders(void);
    void updateRemainingOrders(void);
    int orders_for_pickup=0;
    std::vector<CPizzaOrder> PizzaOrdersReadyPickup;
    int address = 0;
  private:
    int orders_left_behind=0;
    
};

// main class of turtlebot
class Turtlebot3Drive
{
 public:
  Turtlebot3Drive();    // constructor
  virtual ~Turtlebot3Drive();   // destructor
  bool init();          
  virtual bool controlLoop() = 0;

 protected:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers

  // Variables
  double escape_range_;
  int selected_movement_direction;
  int address_check;
  geometry_msgs::Twist cmd_vel;

  // Function prototypes
  void updatecommandVelocity(const double& linear, const double& angular);

  CCamera aCamera;
  CLaser aLaser;
  COdom aOdom;
};

class CPizzaDeliverer: public Turtlebot3Drive
{
  public:
    CPizzaDeliverer(CPizzaStore* WhichPizzaStore);
    ~CPizzaDeliverer();
    bool controlLoop(void);
    void pickupPizza(void);
    void deliverPizza(void);

  private:
    int orders_pickedup = 0;
    int orders_delivered;
    int total_pizzas_on_robot=0;
    CPizzaStore* MyPizzaStore;
    CPizzaOrder PizzaOrders[8];
    bool ready_to_pickup_orders = true;
};

#endif // TURTLEBOT3_DRIVE_H_
