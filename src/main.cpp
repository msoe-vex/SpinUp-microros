#include "main.h"
#include "clock_gettime.h"

#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"

#include "rmw_microros/rmw_microros.h"

#include "std_msgs/msg/header.h"
#include "std_msgs/msg/int32.h"

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
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    pros::lcd::set_text(0, "why");
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
        //pros::lcd::set_text(0, std::to_string(msg.data));
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		msg.data++;
    }
    //pros::lcd::set_text(3, "why part 2 electric boogaloo");

}

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

void setup_controller() {
    pros::Controller master (pros::E_CONTROLLER_MASTER);
    int32_t analog_left_x = master.get_analog(ANALOG_LEFT_X);
    int32_t analog_left_y = master.get_analog(ANALOG_LEFT_Y);
    int32_t analog_right_x = master.get_analog(ANALOG_RIGHT_X);
    int32_t analog_right_y = master.get_analog(ANALOG_RIGHT_Y);
}

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

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

void opcontrol() {
	pros::lcd::set_text(2, "test123");
    
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
		pros::lcd::set_text(1, "timer initted");
	}

    // create node
	RCCHECK(rclc_node_init_default(&node, "freertos_int32_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_best_effort(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"freertos_int32_publisher"));

    timer = rcl_get_zero_initialized_timer();
	// create timer,
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;

    std::uint32_t now = pros::millis();
	while (true) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        pros::lcd::set_text(2, std::to_string(msg.data));
	    //rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        pros::Task::delay_until(&now, 100);
        msg.data++;
	}

	// free resources
	//RCCHECK(rcl_publisher_fini(&publisher, &node))
	//RCCHECK(rcl_node_fini(&node))
    pros::lcd::set_text(2, "huh?");


	/*pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(2);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = right;
		pros::delay(20);
	}*/
}
