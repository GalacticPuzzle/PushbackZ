#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

//initialize motors
pros::MotorGroup left_motor_group({-11, -12, -13}); // left motors on ports 1, 2, 3
pros::MotorGroup right_motor_group({16, 18, 17}); // right motors on ports 4, 5, 6

// initialize conveyor
pros::MotorGroup intake_motor_group({19});

pros::MotorGroup back_conveyor_motor_group({1});

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
&right_motor_group, // right motor group
11.25, // 11.25 inch track width
lemlib::Omniwheel::NEW_325, // using 2.75" omnis
450, // drivetrain rpm is 360
2 // horizontal drift is 2 (for now)
);


// create a v5 rotation sensor on port 1
// imu
pros::Imu imu(15);

pros::Rotation HorizontalOdom(20);
//pros::Rotation VerticalOdom(7);


pros::adi::DigitalOut wing('C');
pros::adi::DigitalOut middle('B');
pros::adi::DigitalOut scraper('A');

lemlib::TrackingWheel HorizontalWheel(&HorizontalOdom, lemlib::Omniwheel::NEW_2,-3.5);
//lemlib::TrackingWheel VerticalWheel(&VerticalOdom, lemlib::Omniwheel::NEW_2,1);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
&HorizontalWheel, // horizontal tracking wheel 1
nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
&imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(11, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
// lemlib::ControllerSettings angular_controller(8, // proportional gain (kP)
// 0, // integral gain (kI)
// 75, // derivative gain (kD)
// 3, // anti windup
// 1, // small error range, in degrees
// 100, // small error range timeout, in milliseconds
// 3, // large error range, in degrees
// 500, // large error range timeout, in milliseconds
// 0 // maximum acceleration (slew)
// );

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              14, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
lateral_controller, // lateral PID settings
angular_controller, // angular PID settings
sensors // odometry sensors
);
//auton selector
int selectedAuton = 1;



// initialize function. Runs on program startup
void initialize() {
pros::lcd::initialize(); // initialize brain screen
chassis.calibrate(); // calibrate sensors
// print position to brain screen
pros::Task screen_task([&]() {
while (true) {

// print robot location to the brain screen
pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
// delay to save resources
pros::delay(20);
}
});
}

/**
* A callback function for LLEMU's center button.
*
* When this callback is fired, it will toggle line 2 of the LCD text between
* "I was pressed!" and nothing.
*/
void on_center_button() {
static bool pressed = false;
pressed = !pressed;
if (pressed) {
pros::lcd::set_text(2, "I was pressed!");
} else {
pros::lcd::clear_line(2);
}
}



void autonomous() {
switch (selectedAuton) {
case 0: //left side

// TEST TEST TEST TEST





case 1:
chassis.setPose(0,0,0);
chassis.moveToPoint(0, 36, 1500,{.maxSpeed=55}, true);
chassis.turnToHeading(90, 1000);
pros::delay(500);
scraper.set_value(true);
intake_motor_group.move(127);
chassis.moveToPoint(14.1, 42, 8000,{.maxSpeed=60}, true);
pros::delay(1500);
chassis.moveToPoint(14.1, 42, 8000,{.maxSpeed=60}, true);
pros::delay(500);
chassis.moveToPoint(-16.5, 44, 1000,{.forwards=false},true);
chassis.waitUntilDone();
back_conveyor_motor_group.move(127);
pros::delay(500);
wing.set_value(true);
pros::delay(800);
wing.set_value(false);



// default:
// chassis.setPose(0,0,0);
// intake_motor_group.move(127);
// chassis.moveToPoint(18.6, 14, 1000, {}, true);
// chassis.waitUntilDone();
// chassis.moveToPoint(-3, -40, 1500,{},true);
// chassis.turnToHeading(270, 1000);
// scraper.set_value(true);
// pros::delay(300);
// chassis.moveToPoint(-13, -40, 2500,{},true);
// chassis.moveToPoint(15.5,-40,1000, {.forwards=false},true);
// chassis.waitUntilDone();
// back_conveyor_motor_group.move(127);


//SKILLS
// chassis.setPose(0,0,0);
// pros::delay(200);
// wing.set_value(true);
// pros::delay(500);
// chassis.moveToPoint(0, 45, 2000);













// case 1: //right side
// chassis.setPose(0,0,0);
// chassis.moveToPoint(0, 31.6, 1500, {.maxSpeed = 60}, false);
// chassis.turnToHeading(268, 1000);
// pros::delay(500);
// scraper.set_value(true);
// intake_motor_group.move(127);

// chassis.moveToPoint(-13.95, 31, 1500, {.maxSpeed = 50}, false);
// pros::delay(200);
// chassis.moveToPoint(-13.95, 31.0, 1500, {.maxSpeed = 60}, false);
// pros::delay(1000);
// chassis.moveToPoint(-14.1, 31.0, 1500, {.maxSpeed = 60}, false);
// pros::delay(1000);
// chassis.moveToPoint(-13.7, 31.0, 1500, {.maxSpeed = 60}, false);
// pros::delay(1000);
// intake_motor_group.move(0);
// chassis.moveToPoint(36, 30, 1500, {.forwards = false, .maxSpeed = 60}, false);
// intake_motor_group.move(127);
// back_conveyor_motor_group.move(127);



case 2:
default:
break;} 
}


pros::Controller controller(pros::E_CONTROLLER_MASTER);



void opcontrol() {
//vars
bool prevA = false;
bool prevB = false;
bool prevY = false;
bool prevX = false;
bool prevWing = false;

bool aToggle = false;
bool bToggle = false;
bool yToggle = false;
bool xToggle = false;
bool wingToggle = false;

int middleRunTime; //shitty code, fix later
bool middleMacroActive;
bool runMiddleMacro;

//loop forever
while (true) {
// get analog stick and button inputs
int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
int rightShoulder1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
int rightShoulder2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
int leftShoulder1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
int leftShoulder2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

bool buttonA = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
bool buttonB = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
bool buttonY = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
bool buttonX = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

// drive chassis
chassis.arcade(leftY, rightX);
// run conveyor
//intake_group.move((rightShoulder1 * 127) + (-rightShoulder2 * 127));
if(leftShoulder2){ //score
back_conveyor_motor_group.move(127);
intake_motor_group.move(127);
} else if (rightShoulder2){ //intake
back_conveyor_motor_group.move(-5);
intake_motor_group.move(127);
} else if (rightShoulder1){ //outtake
back_conveyor_motor_group.move(-5);
intake_motor_group.move(-127);
} else { //idle
back_conveyor_motor_group.move(-5);
intake_motor_group.move(0);
}

//middle goal

// toggle-based pneumatics
if (buttonB && !prevB) {
bToggle = !bToggle;
}
if (buttonA && !prevA) {
aToggle = !aToggle;
}
if (buttonY && !prevY) {
yToggle = !yToggle;
}
if (buttonX && !prevX) {
xToggle = !xToggle;
}

if (leftShoulder1 && !prevWing) {
wingToggle = !wingToggle;
}

// set pneumatic states
wing.set_value(wingToggle);
scraper.set_value(xToggle);
middle.set_value(buttonY);



// update previous button states
prevB = buttonB;
prevA = buttonA;
prevY = buttonY;
prevX = buttonX;
prevWing = leftShoulder1;


// delay to save resources
pros::delay(25);
}
}