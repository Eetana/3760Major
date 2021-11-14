#include "turtlebot3_gazebo/turtlebot3_drive.h"
#include <string.h>

Turtlebot3Drive::Turtlebot3Drive()
 : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  //ROS_INFO("TurtleBot3 Simulation Node Init");
  //auto ret = init();
  //ROS_ASSERT(ret);
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  //updatecommandVelocity(0.0, 0.0);
  //ros::shutdown();
}

CPizzaDeliverer::CPizzaDeliverer(CPizzaStore* WhichPizzaStore)
	: MyPizzaStore(WhichPizzaStore)
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  auto ret = init();
  ROS_ASSERT(ret);
}

CPizzaDeliverer::~CPizzaDeliverer()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown(); 
}

/*******************************************************************************
* Init function
*******************************************************************************/
void CLaser::initLaser(ros::NodeHandle *nh_){
    laser_scan_sub_  = nh_->subscribe("scan", 10, &CLaser::laserScanMsgCallBack, this);
    check_forward_dist_ = 0.5;
    check_side_dist_    = 0.65; //0.5
}

void COdom::initOdom(ros::NodeHandle *nh_){
    odom_sub_  = nh_->subscribe("odom", 10, &COdom::odomMsgCallBack, this);
    tb3_pose_ = 0.0;
    prev_tb3_pose_ = 0.0;
}

void CCamera::initCamera(ros::NodeHandle *nh_){

  //Cam subscribe 
  tag_detections_sub_ = nh_->subscribe("tag_detections", 10, &CCamera::tagDetectMsgCallBack, this);
}

bool Turtlebot3Drive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  aCamera.initCamera(&nh_);
  aLaser.initLaser(&nh_);
  aOdom.initOdom(&nh_);

  return true;
}

CPizzaOrder createOrder(void){
	CPizzaOrder NewOrder;
	NewOrder.no_pizza_in_order = (rand()%4)+1;
	//NewOrder.pizza_add = (rand()%8)+1;
	NewOrder.pizza_add = 3;
	NewOrder.delivered_status = false;
	return NewOrder;
}

void CPizzaDeliverer::deliverPizza(void){
  //Check if robot reaches address
  int i = 0;
  orders_delivered = 0;
  for (i = 0; i < orders_pickedup; i++){
	if ((PizzaOrders[i].pizza_add == aCamera.tag_id) && (PizzaOrders[i].delivered_status != true)){
		printf("Delivered: %d Pizzas to address: %d\n", PizzaOrders[i].no_pizza_in_order, PizzaOrders[i].pizza_add);
		total_pizzas_on_robot = total_pizzas_on_robot-PizzaOrders[i].no_pizza_in_order;		
		PizzaOrders[i].no_pizza_in_order = 0;
		PizzaOrders[i].delivered_status = true;
		orders_delivered++;
	}
  }
  orders_pickedup = orders_pickedup-orders_delivered;
  ready_to_pickup_orders = true;
}

void CPizzaStore::prepareOrders(){
  	//Randomly generate no. of orders to be picked up
	if (orders_left_behind == 0){	
		printf("Zero orders left behind\n");
		orders_for_pickup = ((rand()%4)+1);
	}
	else{
		printf("Orders left behind: %d\n", orders_left_behind);
		orders_for_pickup = ((rand()%(4-orders_left_behind))+1)+orders_left_behind;
		
	}
	printf("Orders for pickup: %d\n", orders_for_pickup);
  	int current_order = 0;
  	auto ptrPizzaOrdersReadyPickup = PizzaOrdersReadyPickup.begin();
        for(current_order = orders_left_behind; current_order < orders_for_pickup; current_order++){
		ptrPizzaOrdersReadyPickup = PizzaOrdersReadyPickup.begin();
  		ptrPizzaOrdersReadyPickup = PizzaOrdersReadyPickup.insert(ptrPizzaOrdersReadyPickup,CPizzaOrder());

		PizzaOrdersReadyPickup[0] = createOrder();

		printf("Ready to Pick Up: %d Pizzas, Deliver to address: %d\n", PizzaOrdersReadyPickup[0].no_pizza_in_order, PizzaOrdersReadyPickup[0].pizza_add);
  	}
}

void CPizzaDeliverer::pickupPizza(){
  	if (aCamera.tag_id == MyPizzaStore->address){
		ready_to_pickup_orders = false;
		
		int i = 0;
		for(i = 0; i < MyPizzaStore->orders_for_pickup; i++){
			//Check if robot can hold next order's pizzas
			if ((total_pizzas_on_robot + MyPizzaStore->PizzaOrdersReadyPickup[MyPizzaStore->PizzaOrdersReadyPickup.size()-1].no_pizza_in_order) <= 8){
				PizzaOrders[i]  = MyPizzaStore->PizzaOrdersReadyPickup[MyPizzaStore->PizzaOrdersReadyPickup.size()-1];
				printf("Picked Up: %d Pizzas, Deliver to address: %d\n", PizzaOrders[i].no_pizza_in_order, PizzaOrders[i].pizza_add);
				orders_pickedup++;
		        	total_pizzas_on_robot = total_pizzas_on_robot + PizzaOrders[i].no_pizza_in_order;
				MyPizzaStore->PizzaOrdersReadyPickup.resize(MyPizzaStore->PizzaOrdersReadyPickup.size()-1);
			}
			else{
				break;
			}
		}
		MyPizzaStore->updateRemainingOrders();
	}
}

void CPizzaStore::updateRemainingOrders(void){
	orders_left_behind = PizzaOrdersReadyPickup.size();
	printf("Orders left behind: %d\n", orders_left_behind);
}

void CCamera::tagDetectMsgCallBack(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
	//printf("tag_detections\n");
        //printf("id: %d\n", msg->detections[0].id[0]);
        //printf("size: %f\n", msg->detections[0].size[0]);
        //printf("pose x: %f\n", msg->detections[0].pose.pose.pose.position.x);
	//printf("pose y: %f\n", msg->detections[0].pose.pose.pose.position.y);
	//printf("pose z: %f\n", msg->detections[0].pose.pose.pose.position.z);
	//If there is an apriltag detecting, store these values
	//if (msg->detections[0].pose.pose.pose.position.x != NULL){
	if (msg->detections.empty() != 1){
		//printf("array is empty\n");
		x_tag_pose = msg->detections[0].pose.pose.pose.position.x;
        	y_tag_pose = msg->detections[0].pose.pose.pose.position.y;
        	z_tag_pose = msg->detections[0].pose.pose.pose.position.z;
		tag_id = msg->detections[0].id[0];
		tag_detected = true;	
		
	}
	else{
		tag_detected = false;
	}
}

void COdom::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	tb3_pose_ = atan2(siny, cosy);
}

void CLaser::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[14] = {0,15,30,45,60,75,90,180,270,285,300,315,330,345};

  for (int num = 0; num < 14; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3Drive::updatecommandVelocity(const double& linear, const double& angular)
{

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
CLaser::movementDirection CLaser::checkMovementDirection(void){
  if(scan_data_[CENTER] > 0.65 && scan_data_[RIGHT_300] > check_side_dist_ && scan_data_[RIGHT_285]> check_side_dist_ && scan_data_[RIGHT_330] > check_side_dist_){

	return right_turn;
  }
  else if((scan_data_[CENTER]>0.65) && (scan_data_[RIGHT_285]> 0.35)){

	return straight;
  }
  else if ((scan_data_[CENTER] < 0.65) && (scan_data_[RIGHT_270] < 0.65 || scan_data_[RIGHT_345] < 0.65 || scan_data_[RIGHT_300] < 0.65)){

       return left_turn;
  }
  else if ((scan_data_[CENTER] < 2) && (scan_data_[RIGHT_270] < check_side_dist_ || scan_data_[RIGHT_345] < check_side_dist_ || scan_data_[RIGHT_300] < check_side_dist_)){

      return left_collision_turn;
  }
}

CCamera::addressDetected CCamera::checkAddress(int pizza_shop_address){
	if(tag_detected == true){
		if ((x_tag_pose < 1) && (z_tag_pose < 1)){
			if (tag_id == pizza_shop_address){
				return pizza_shop;		
			}
			else{
				return not_pizza_shop;
			}
		}
		else{
			return no_address;
		}
	}
	else{
		return no_address;
	}
}

bool CPizzaDeliverer::controlLoop()
{
  static uint8_t turtlebot3_state_num = 0;
  static bool FLAG =true;
  static bool not_moving = false;
  switch(turtlebot3_state_num)
  {
    case GET_TB3_DIRECTION:
    aOdom.prev_tb3_pose_ = aOdom.tb3_pose_; 
    address_check = aCamera.checkAddress(MyPizzaStore->address);
    if (address_check == 1){
	if (ready_to_pickup_orders == true){
		MyPizzaStore->prepareOrders();
		pickupPizza();
	}
    }
    else if(address_check == 2){
	deliverPizza();
    }
    else if(address_check == 0){
        selected_movement_direction = aLaser.checkMovementDirection();
        if(selected_movement_direction == 0){
        	turtlebot3_state_num = TB3_RIGHT_TURN;
	}
        else if(selected_movement_direction == 1){
		turtlebot3_state_num = TB3_DRIVE_FORWARD;
	}
	else if(selected_movement_direction == 2){
		turtlebot3_state_num = TB3_LEFT_TURN;
	}
	else if(selected_movement_direction == 3){
		turtlebot3_state_num = TB3_LEFT_TURN;
	}
        
    }
    else{
      not_moving = true;
    }
    break;

    case TB3_DRIVE_FORWARD:
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      if (fabs(aOdom.prev_tb3_pose_ - aOdom.tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      break;

    case TB3_LEFT_TURN:
      if (fabs(aOdom.prev_tb3_pose_ - aOdom.tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      break;  

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  //Turtlebot3Drive turtlebot3_drive;
   
  CPizzaStore aPizzaStore;
  //Turtlebot3Drive* turtlebot3_drive = new CPizzaDeliverer(&aPizzaStore);
  CPizzaDeliverer aPizzaDeliverer(&aPizzaStore);

  ros::Rate loop_rate(125);

  //srand(4322);

  while (ros::ok())
  {
    aPizzaDeliverer.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
