/*
 * DDD.cpp
 *
 *  Created on: 7 Aug 2023
 *      Author: jondurrant
 */

#include "Busterbot.h"
#include <cstdio>
#include <cmath>
#include "uRosBridge.h"

using namespace Eigen;

Busterbot::Busterbot() {
	xMotorsOdom.x=0.0;
	xMotorsOdom.y=0.0;
	xMotorsOdom.a=0.0;

	xBusterbotOdom.x=0.0;
	xBusterbotOdom.y=0.0;
	xBusterbotOdom.a=0.0;
}

Busterbot::~Busterbot() {
	// TODO Auto-generated destructor stub
}

void Busterbot::setMotorsAgent(MotorsAgent *p){
	pMotorsAgent = p;
}


/***
 * Run loop for the agent.
 */
void Busterbot::run(){
	//printf("Busterbot Running\n");
	setupOdomMsg();
	for (;;){
		if (pMotorsAgent != NULL){
			uint32_t timeSinceTwise = to_ms_since_boot (get_absolute_time ()) - xLastTwistTimestamp;
			if (timeSinceTwise > MAX_TWIST_TIME_MS){
				robotStop();
				printf("STOPPED - twist msg not received in %d MS\n", MAX_TWIST_TIME_MS);
			}
			updateOdom();

			/*
			printf("X: %.3f Y: %.3f A: %.3f Deg: %.3f\n",
					xBusterbotOdom.x,
					xBusterbotOdom.y,
					xBusterbotOdom.a,
					xBusterbotOdom.a/M_PI * 180.0);
			*/
			 

			publishOdom();

		}

		vTaskDelay(500);
	}
}

/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE Busterbot::getMaxStackSize(){
	return 1024;
}

void Busterbot::updateOdom(){

	//printf("Update Odom\n");

	double l = pMotorsAgent->getMotor(0)->getDeltaRadians();
	double r = pMotorsAgent->getMotor(1)->getDeltaRadians();

	l=l*WHEEL_RADIUS;
	r=r*WHEEL_RADIUS* -1.0;

	double avgDist = (r+l)/2.0;
	double angle = asin((r-l)/WHEELS_SEP);
	double deltaX = cos(angle)* avgDist;
	double deltaY = sin(angle)* avgDist;

	xMotorsOdom.x += deltaX;
	xMotorsOdom.y += deltaY;
	xMotorsOdom.a += angle;

	xBusterbotOdom.x = xMotorsOdom.x + (cos(angle) * WHEELS_OFFSET);
	xBusterbotOdom.y = xMotorsOdom.y + (sin(angle) * WHEELS_OFFSET);
	xBusterbotOdom.a = angle;

	uint32_t now = to_ms_since_boot( get_absolute_time()    );
	double seconds = (double)(now - xLastVelocityTime) / 1000;
	xLastVelocityTime = now;
	xBusterbotVelocity.x = deltaX /seconds;
	xBusterbotVelocity.y = deltaY /seconds;
	xBusterbotVelocity.a = angle /seconds;

	/*
	printf("L: %.3f r: %0.3f dist: %.3f ang: %.3f "
			   "x: %.3f y: %.3f a: %.3f "
				"MX: %.3f MY: %.3f MA: %.3f "
				"X: %.3f, Y: %.3f A: %.3f\n",
				l, r, avgDist, angle,
				deltaX, deltaY, angle/M_PI * 180.0,
				xMotorsOdom.x, xMotorsOdom.y, xMotorsOdom.a/M_PI * 180.0,
				xDDDOdom.x, xDDDOdom.y, xDDDOdom.a/M_PI * 180.0
				);
	*/

}

void Busterbot::publishOdom(){

	//printf("Publish Odom\n");

	//Update header
	int64_t time = rmw_uros_epoch_nanos();
	xOdomMsg.header.stamp.sec = time / 1000000000;
	xOdomMsg.header.stamp.nanosec = time % 1000000000;

	//POSE
	xOdomMsg.pose.pose.position.x = xBusterbotOdom.x;
	xOdomMsg.pose.pose.position.y = xBusterbotOdom.y;
	Quaterniond q;
	Matrix3d m;
	m = AngleAxisd(0.0, 		Vector3d::UnitX())
	  * AngleAxisd(0.0,  		Vector3d::UnitY())
	  * AngleAxisd(xBusterbotOdom.a, 	Vector3d::UnitZ());
	q = m;
	xOdomMsg.pose.pose.orientation.x = q.x();
	xOdomMsg.pose.pose.orientation.y = q.y();
	xOdomMsg.pose.pose.orientation.z = q.z();
	xOdomMsg.pose.pose.orientation.w = q.w();

	//TWIST
	xOdomMsg.twist.twist.linear.x 	= xBusterbotVelocity.x;
	xOdomMsg.twist.twist.linear.y 	= xBusterbotVelocity.y;
	xOdomMsg.twist.twist.angular.z 	= xBusterbotVelocity.a;


	if (!uRosBridge::getInstance()->publish(&xPubOdom,
			&xOdomMsg,
			this,
			NULL)){
		printf("Odom pub failed\n");
	} else{
		//printf("Odom Pub success\n");
	}

}

void Busterbot::setupOdomMsg(){
	nav_msgs__msg__Odometry__init(&xOdomMsg);
	if (!rosidl_runtime_c__String__assign(
			&xOdomMsg.header.frame_id, "odom")){
			printf("ERROR: Odom frameID assignment failed\n");
	}
	if (!rosidl_runtime_c__String__assign(
			&xOdomMsg.child_frame_id, "base_link")){
			printf("ERROR: Odom frameID assignment failed\n");
	}

	//POSE
	xOdomMsg.pose.pose.position.x = 0.0;
	xOdomMsg.pose.pose.position.y = 0.0;
	xOdomMsg.pose.pose.position.z = 0.0;
	xOdomMsg.pose.pose.orientation.x = 0.0;
	xOdomMsg.pose.pose.orientation.y = 0.0;
	xOdomMsg.pose.pose.orientation.z = 0.0;
	xOdomMsg.pose.pose.orientation.w = 0.0;

	//TWIST
	xOdomMsg.twist.twist.linear.x = 0.0;
	xOdomMsg.twist.twist.linear.y = 0.0;
	xOdomMsg.twist.twist.linear.z = 0.0;
	xOdomMsg.twist.twist.angular.x = 0.0;
	xOdomMsg.twist.twist.angular.y = 0.0;
	xOdomMsg.twist.twist.angular.z = 0.0;

}

void Busterbot::setupTwistMsg(){
	printf("setupTwistMsg - Busterbot.cpp\n");
	bool resp = geometry_msgs__msg__Twist__init(&xTwistMsg);
	if (!resp){
		printf("ERROR: Twist init failed\n");
	}	else {
		printf("Twist init OK\n");
	}
}


/***
 * Create the publishing entities
 * @param node
 * @param support
 */
void Busterbot::createEntities(rcl_node_t *node, rclc_support_t *support){

	// create joint_states publisher
	if (pMotorsAgent != NULL){
		pMotorsAgent->createEntities(node, support);
	}

	printf("Busterbot.cpp >> creating entities ... \n");

	printf("Initting odom publisher entity\n");
	rcl_ret_t dummy2 = rclc_publisher_init_default(
		&xPubOdom,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
		"/busterbot/odom"
	);


	if (dummy2 != RCL_RET_OK){
		printf("ERROR: Odom publisher failed\n");
	} else {
		printf("Odom publisher created\n");
	}

	// rclc_subscription_init_best_effort or rclc_subscription_init_default
	printf("Initting cmd_vel subscriber entity\n");
	//rcl_subscription_t my_sub = rcl_get_zero_initialized_subscription();

	// best_effort or default
	//RCCHECK(rclc_subscription_init_best_effort(
	RCCHECK(rclc_subscription_init_default(
	//rcl_ret_t dummy1 = rclc_subscription_init_default(
		&xSubTwist,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"/busterbot/cmd_vel"
	));

	if (dummy1 != RCL_RET_OK){
		printf("ERROR: Twist subscription failed\n");
	} else{
		printf("Twist subscription created\n");
	}




}

/***
 * Destroy the publishing entities
 * @param node
 * @param support
 */
void Busterbot::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	if (pMotorsAgent != NULL){
		pMotorsAgent->destroyEntities(node, support);
	}
	rcl_ret_t dummy1 = rcl_subscription_fini(&xSubTwist, 	node);
}

/***
 * Provide a count of the number of entities
 * @return number of entities >=0
 */
uint Busterbot::getCount(){
	uint res = 2;
	if (pMotorsAgent != NULL){
		res += pMotorsAgent->getCount();
	}
	return res;
}

/***
 * Return the number of handles needed by the executor
 * @return
 */
uint Busterbot::getHandles(){
	uint res = 1;
	if (pMotorsAgent != NULL){
		res += pMotorsAgent->getHandles();
	}
	return res;
}


/***
 * Add subscribers, guards and timers to the executor
 * @param executor
 */
void Busterbot::addToExecutor(rclc_executor_t *executor){

	if (pMotorsAgent != NULL){
		pMotorsAgent->addToExecutor(executor);
	}

	printf("Adding twist subscription to executor - Busterbot.cpp\n");
	buildContext(&xSubTwistContext, NULL);
	rclc_executor_add_subscription_with_context(
			executor,
			&xSubTwist,
			&xTwistMsg,
			uRosEntities::subscriptionCallback,
			&xSubTwistContext,
			ON_NEW_DATA);

	// Note - now must spin the executor to receive the messages - see uRosBridge.cpp run

}


/***
 * Handle subscription msg
 * @param msg
 * @param localContext
 */
void Busterbot::handleSubscriptionMsg(const void* msg, uRosSubContext_t* context){

	printf("Handling twist subscription - Busterbot.cpp\n");

	if (pMotorsAgent == NULL){
		return;
	}

	if (context == &xSubTwistContext){
		geometry_msgs__msg__Twist * pTwistMsg = (geometry_msgs__msg__Twist *) msg;
		double circum = WHEEL_RADIUS * 2.0 * M_PI;

#ifdef TWIST_DEBUG
		printf("TWIST x: %.3f  z: %.3f\n",
				pTwistMsg->linear.x,
				pTwistMsg->angular.z
				);
#endif //TWIST_DEBUG

		xLastTwistTimestamp = to_ms_since_boot (get_absolute_time ());

		//Stop
		if (pTwistMsg->linear.x == 0.0){
			printf("linear x is 0 - so stopping\n");
			//Have not move linearly to turn
			pMotorsAgent->setSpeedRadPS(0, 0.0, true);
			pMotorsAgent->setSpeedRadPS(1, 0.0, false);
			return;
		}

		// FWD and Backwards
		if (pTwistMsg->angular.z == 0.0){
			printf("angular z is 0 - going straight\n");
			double rps = (pTwistMsg->linear.x / circum) * (2 * M_PI);
			bool cw = true;
			if (rps < 0.0){
				cw = false;
				rps = rps * -1;
			}
			pMotorsAgent->setSpeedRadPS(0, rps, cw);
			pMotorsAgent->setSpeedRadPS(1, rps, cw);

#ifdef TWIST_DEBUG
			printf("LINEAR TWIST %.3f mps becomes %.3f Rad ps\n",
					pTwistMsg->linear.x,
					rps
					);
#endif //TWIST_DEBUG
		} else {
			//ARC
			bool fwd = (pTwistMsg->linear.x > 0.0);
			bool cw = (pTwistMsg->angular.z > 0.0);
			double a = fabs(pTwistMsg->angular.z);
			double arc = a/ (M_PI * 2);
			double fullCircleCircum = (pTwistMsg->linear.x / arc);
			double radius = fullCircleCircum / ( 2.0 * M_PI);

			double speedA = (radius + WHEELS_SEP/4) * (2 * M_PI) * arc;
			double speedB = (radius - WHEELS_SEP/4) * (2 * M_PI) * arc;

			double rpsA = (speedA / circum) * (2 * M_PI);
			double rpsB = (speedB / circum) * (2 * M_PI);

			if (fwd){
				if (!cw){
					pMotorsAgent->setSpeedRadPS(0, rpsA, !fwd);
					pMotorsAgent->setSpeedRadPS(1, rpsB,  fwd);
				} else {
					pMotorsAgent->setSpeedRadPS(0, rpsB, !fwd);
					pMotorsAgent->setSpeedRadPS(1, rpsA,  fwd);
				}
			} else {
				if (cw){
					pMotorsAgent->setSpeedRadPS(0, rpsA,  fwd);
					pMotorsAgent->setSpeedRadPS(1, rpsB, !fwd);
				} else {
					pMotorsAgent->setSpeedRadPS(0, rpsB,  fwd);
					pMotorsAgent->setSpeedRadPS(1, rpsA, !fwd);
				}
			}

#ifdef TWIST_DEBUG
			printf("ROTATE TWIST %.3f mps at %.3f rad ps "
					"becomes %.3f and %.3f Rad ps\n",
					pTwistMsg->linear.x,
					pTwistMsg->angular.z,
					rpsA,
					rpsB
					);
			printf("ROTATE Detail: Radius %.3f Full Circum %.3f Arc %.3f speed A %.3f B %.3f\n",
					radius,
					fullCircleCircum,
					arc,
					speedA,
					speedB);
#endif //TWIST_DEBUG

		}


	}
}


void Busterbot::robotStop(){
	xLastTwistTimestamp = to_ms_since_boot (get_absolute_time ());

	if (pMotorsAgent != NULL){
		pMotorsAgent->setSpeedRadPS(0, 0.0, true);
		pMotorsAgent->setSpeedRadPS(1, 0.0, false);
	}
}

