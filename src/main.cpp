#include "main.h"
#include "clock_gettime.h"

#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"

#include "rmw_microros/rmw_microros.h"

#include "micro_ros_utilities/type_utilities.h"

#include "std_msgs/msg/header.h"
#include "std_msgs/msg/string.h"
#include "sensor_msgs/msg/joy.h"

#include <stdio.h>
#include <unistd.h>
#include <time.h>

// --- micro-ROS Transports ---
extern "C" bool v5_serial_open(struct uxrCustomTransport * transport);
extern "C" bool v5_serial_close(struct uxrCustomTransport * transport);
extern "C" size_t v5_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
extern "C" size_t v5_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
// ----------------------------

#define STRING_BUFFER_LEN 100

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){pros::lcd::print(0, "Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){pros::lcd::print(1, "Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
sensor_msgs__msg__Joy msg;
std_msgs__msg__String sub_msg;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
/*void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}*/

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	msg.axes.capacity = 4;
	msg.axes.data = (float*) malloc(msg.axes.capacity * sizeof(float));
	msg.buttons.capacity = 12;
	msg.buttons.data = (int32_t*) malloc(msg.buttons.capacity * sizeof(int32_t));
	//pros::lcd::set_text(0, PROS_VERSION_STRING);

	//pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void create_joy_message() {
	msg.axes.data[0] = (float) pros::c::controller_get_analog(CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_X);
	msg.axes.data[1] = (float) pros::c::controller_get_analog(CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y);
	msg.axes.data[2] = (float) pros::c::controller_get_analog(CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X);
	msg.axes.data[3] = (float) pros::c::controller_get_analog(CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_Y);
	msg.axes.size = 4;
	
	msg.buttons.data[0] = pros::c::controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT);
	msg.buttons.data[1] = pros::c::controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN);
	msg.buttons.data[2] = pros::c::controller_get_digital(CONTROLLER_MASTER, DIGITAL_UP);
	msg.buttons.data[3] = pros::c::controller_get_digital(CONTROLLER_MASTER, DIGITAL_RIGHT);
	
	msg.buttons.data[4] = pros::c::controller_get_digital(CONTROLLER_MASTER, DIGITAL_A);
	msg.buttons.data[5] = pros::c::controller_get_digital(CONTROLLER_MASTER, DIGITAL_B);
	msg.buttons.data[6] = pros::c::controller_get_digital(CONTROLLER_MASTER, DIGITAL_X);
	msg.buttons.data[7] = pros::c::controller_get_digital(CONTROLLER_MASTER, DIGITAL_Y);
	
	msg.buttons.data[8] = pros::c::controller_get_digital(CONTROLLER_MASTER, DIGITAL_L1);
	msg.buttons.data[9] = pros::c::controller_get_digital(CONTROLLER_MASTER, DIGITAL_L2);
	msg.buttons.data[10] = pros::c::controller_get_digital(CONTROLLER_MASTER, DIGITAL_R1);
	msg.buttons.data[11] = pros::c::controller_get_digital(CONTROLLER_MASTER, DIGITAL_R2);
	msg.buttons.size = 12;
	
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC, &tp);
	
	msg.header.stamp.nanosec = (int32_t) tp.tv_nsec;
	msg.header.stamp.sec = (int32_t) tp.tv_sec;
}

void subscription_callback(const void * msgin) {
  // Cast received message to used type
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;

  // Process message
  //pros::delay(20);
  //pros::lcd::print(4, "Received: %d\n", msg->data);
}

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

void opcontrol() {
	pros::delay(20);
    allocator = rcl_get_default_allocator();

    rmw_uros_set_custom_transport(
        true,
        NULL,
        v5_serial_open,
        v5_serial_close,
        v5_serial_write,
        v5_serial_read
    );

    // create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
	
	// Sync timeout
	const int timeout_ms = 1000;

	// Synchronize time with the agent
	rmw_uros_sync_session(timeout_ms);
	
	if (rmw_uros_epoch_synchronized()) {
		struct timespec ts;
		ts.tv_sec = rmw_uros_epoch_millis()/1000;
		ts.tv_nsec = rmw_uros_epoch_nanos();
		clock_settime(CLOCK_REALTIME, &ts);
		pros::lcd::set_text(6, "timer initted");
	}

    // create node
	node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "main_node", "", &support));

    /*timer = rcl_get_zero_initialized_timer();
	// create timer,
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));*/
	
	// create publisher
	//publisher = rcl_get_zero_initialized_publisher();

	std_msgs__msg__String__init(&sub_msg);
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	subscriber = rcl_get_zero_initialized_subscription();

	RCCHECK(rclc_subscription_init_best_effort(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
		"chatter"));
	
	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&subscriber,
		&sub_msg,
		&subscription_callback,
		ON_NEW_DATA));
	
	pros::delay(20);
	//RCCHECK(rclc_executor_add_timer(&executor, &timer));
	publisher = rcl_get_zero_initialized_publisher();
	RCCHECK(rclc_publisher_init_best_effort(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
		"joy_publisher"));
	
    //RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

    std::uint32_t now = pros::millis();
	while (true) {
		create_joy_message();
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
	    //RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5000)));
		pros::Task::delay_until(&now, 20);
	}

	// free resources
	//RCCHECK(rcl_publisher_fini(&publisher, &node))
	//RCCHECK(rcl_node_fini(&node))

}
