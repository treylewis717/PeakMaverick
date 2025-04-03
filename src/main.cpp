#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

pros::MotorGroup left_motors({-7, -8, -9}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({1, 2, 3}, pros::MotorGearset::blue);

lemlib::Drivetrain dt(&left_motors, &right_motors, 13,
                      lemlib::Omniwheel::NEW_325, 450, 2);

pros::Imu imu(12);

pros::Rotation horizontal(5);
pros::Rotation vertical(-4);

lemlib::TrackingWheel horizontalTW(&horizontal, 2, 0.75);
lemlib::TrackingWheel verticalTW(&vertical, 2, 1);

lemlib::OdomSensors sensors(
    &verticalTW, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontalTW, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       3,   // derivative gain (kD)
                       3,   // anti windup
                       0.1, // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       10   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       80,  // derivative gain (kD)
                       3,   // anti windup
                       .5,  // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       2,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

// create the chassis
lemlib::Chassis chassis(dt,                 // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

void initialize() {
  pros::lcd::initialize();
  chassis.calibrate();
  pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // delay to save resources
      pros::delay(20);
    }
  });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
  right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
  pros::Motor liftM(LIFT_PORT);
  pros::Motor intakeM(INTAKE_PORT);
  pros::Motor ladyBrown(LB_PORT);
  pros::adi::DigitalIn ladyBrownClosed(LIMIT_SWITCH_PORT);
  pros::Distance ringCheck(LIFT_RING_CHECK_PORT);
  pros::Distance distanceSensor(DISTANCE_SENSOR_PORT);
  pros::Controller controller(pros::E_CONTROLLER_MASTER);
  ladyBrown.set_brake_mode_all(pros::MotorBrake::hold);
  pros::adi::DigitalOut clamp(CLAMP_PORT);

  chassis.setPose(0, -12, 0);

  liftM.move(127); // ALIANCE STAKE
  pros::delay(300);

  chassis.moveToPoint(0, 0, 1000);

  chassis.turnToHeading(90, 1000, {}, false);

  chassis.moveToPoint(-26, chassis.getPose().y, 700,
                      {.forwards = false, .maxSpeed = 63},
                      false); // MOGO 1

  clamp.set_value(1);

  chassis.turnToHeading(0, 1000, {}, false);

  intakeM.move(127);

  chassis.moveToPoint(chassis.getPose().x, 24, 1000); // RING 1

  chassis.turnToHeading(-45, 1000, {}, false);

  chassis.moveToPose(chassis.getPose().x - 20, chassis.getPose().y + 48, 0,
                     5000, {}, false); // RING 2

  chassis.moveToPoint(chassis.getPose().x + 3, chassis.getPose().y - 25, 5000,
                      {.forwards = false});

  chassis.turnToPoint(-72, 47, 3000, {}, false); // WALL STAKE 1

  ladyBrown.move_absolute(65, 127);

  chassis.moveToPoint(-72, 47, 1000); // HITS WALL STAKE

  int start = pros::millis();
  int timeSince = 0;

  while (true) {
    if (ringCheck.get_distance() < 50 || timeSince > 3000) {
      break;
    }
    timeSince = pros::millis() - start;
  }

  ladyBrown.move_absolute(720, 127);

  while (ladyBrown.get_position() < 600) {
    pros::delay(10);
  }

  ladyBrown.move_absolute(0, 127);

  pros::delay(1000);

  chassis.moveToPoint(-36, 47, 1000, {.forwards = false});
}

void opcontrol() {}