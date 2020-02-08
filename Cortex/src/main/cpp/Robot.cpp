/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <string>
#include <sstream>
#include <iostream>
#include <ctre/Phoenix.h>
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/ADXRS450_Gyro.h>

//Declarations
TalonSRX srx = {0};

//Motors
//Left Side
TalonFX LeftMotorOne{13};
TalonFX LeftMotorTwo{14};
TalonFX LeftMotorThree{15};
//Right Side
TalonFX RightMotorOne{2};
TalonFX RightMotorTwo{1};
TalonFX RightMotorThree{0};

//PDP
frc::PowerDistributionPanel pdp{0};

//Gyro
//AnalogGyro gyro{0};
frc::ADXRS450_Gyro gyro{frc::SPI::Port::kMXP};

//Joysticks
frc::Joystick JoyAccel1{0}, Xbox{1}, RaceWheel{2};

double JoyY;
double WheelX;
int currentAutoStep;
bool inverted = false;
bool isDelayTimeStampSet;
double delayTimeStamp;

//Acceleration Variables
double accelerationSpeed;
double accelTimeStamp;
double changeInY;
double accelStartSpeed;
double deltaSpeed;
bool isAccelTimeStampSet;

//Functions
void LeftMotorsSpeed(double speed) {
  LeftMotorOne.Set(ControlMode::PercentOutput, -speed);
  LeftMotorTwo.Set(ControlMode::PercentOutput, -speed);
  LeftMotorThree.Set(ControlMode::PercentOutput, -speed);
}
void RightMotorsSpeed(double speed) {
  RightMotorOne.Set(ControlMode::PercentOutput, speed);
  RightMotorTwo.Set(ControlMode::PercentOutput, speed);
  RightMotorThree.Set(ControlMode::PercentOutput, speed);
}

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  srx.Set(ControlMode::PercentOutput, 0);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
    
    //Auto Init
    currentAutoStep = 1;
    isDelayTimeStampSet = false;
    delayTimeStamp = 0;
    LeftMotorOne.SetSelectedSensorPosition(0);
    RightMotorOne.SetSelectedSensorPosition(0);
  }
}

//Autonomous Functions
void goDistance(double inches, double speed) {
  double encoderUnits = inches * 4000/12;
  double averageEncoderValue = (-(LeftMotorOne.GetSelectedSensorPosition()) + RightMotorOne.GetSelectedSensorPosition())/2;
  //double distanceLeft = encoderUnits - ((LeftMotorOne.GetSelectedSensorPosition() + RightMotorOne.GetSelectedSensorPosition()) / 2)
  if (averageEncoderValue < encoderUnits && encoderUnits > 0) {
    LeftMotorsSpeed(speed);
    RightMotorsSpeed(speed);
  }
  else if (averageEncoderValue > encoderUnits && encoderUnits < 0) {
    LeftMotorsSpeed(-speed);
    RightMotorsSpeed(-speed);
  }
  else {
    LeftMotorOne.SetSelectedSensorPosition(0);
    RightMotorOne.SetSelectedSensorPosition(0);
    currentAutoStep = currentAutoStep + 1;
  }
}

void turn(double degrees, double speed){
  double encoderUnits = (degrees * 26000)/360;
  double averageEncoderValue = (LeftMotorOne.GetSelectedSensorPosition() + RightMotorOne.GetSelectedSensorPosition())/2;
  if (averageEncoderValue > -encoderUnits && encoderUnits > 0) {
    LeftMotorsSpeed(speed);
    RightMotorsSpeed(-speed);
  } else if (averageEncoderValue < -encoderUnits && encoderUnits < 0){
    LeftMotorsSpeed(-speed);
    RightMotorsSpeed(speed);
  } else {
    LeftMotorOne.SetSelectedSensorPosition(0);
    RightMotorOne.SetSelectedSensorPosition(0);
    currentAutoStep = currentAutoStep + 1;
  }
}

void delay(double seconds) {
  if (!isDelayTimeStampSet){
    delayTimeStamp = frc::Timer::GetFPGATimestamp();
    isDelayTimeStampSet = true;
  }
  else if (frc::Timer::GetFPGATimestamp() < delayTimeStamp + seconds) {
    LeftMotorsSpeed(0);
    RightMotorsSpeed(0);
    LeftMotorOne.SetSelectedSensorPosition(0);
    RightMotorOne.SetSelectedSensorPosition(0);
  } 
  else {
    currentAutoStep = currentAutoStep + 1;
    isDelayTimeStampSet = false;
  }
}

void stopAll(){
  LeftMotorsSpeed(0);
  RightMotorsSpeed(0);
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
    frc::SmartDashboard::PutNumber("RightEncoderOne", RightMotorOne.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("LeftEncoderOne", LeftMotorOne.GetSelectedSensorPosition());

    switch (currentAutoStep){
      case 1:
      goDistance(24, 0.2);
      break;

      case 2:
      delay(3);         
      break;

      case 3:
      goDistance(24, 0.2);
      break;

      case 4:
      delay(3);
      break;

      case 5:
      turn(180, 0.2);
      break;

      default:
      LeftMotorsSpeed(0);
      RightMotorsSpeed(0);
    }
  }
}

void Robot::TeleopInit() {
  //Reset encoder values
  RightMotorOne.SetSelectedSensorPosition(0);
  LeftMotorOne.SetSelectedSensorPosition(0);
  accelerationSpeed = 0;
  accelTimeStamp = 0;
  changeInY = 0;
  accelStartSpeed = 0;
  deltaSpeed = 0;
}

//Teleop Functions
void accelerate(double percentPerSecond){
  double averageMotorSpeed = (-(LeftMotorOne.GetMotorOutputPercent()) + RightMotorOne.GetMotorOutputPercent())/2;
  //JoyY or averageMotorSpeed < targetSpeed
  if (!isAccelTimeStampSet || ((changeInY > 0 && -JoyAccel1.GetY() - averageMotorSpeed < 0) || (changeInY < 0 && -JoyAccel1.GetY() - averageMotorSpeed > 0))) {
    isAccelTimeStampSet = true;
    accelTimeStamp = frc::Timer::GetFPGATimestamp();
    accelStartSpeed = -JoyAccel1.GetY();
  } else {
    deltaSpeed = (frc::Timer::GetFPGATimestamp() - accelTimeStamp) * percentPerSecond;
    if (-JoyAccel1.GetY() > averageMotorSpeed && -JoyAccel.GetY() > 0.1) {
      accelerationSpeed = accelStartSpeed + deltaSpeed;
    } else if (-JoyAccel1.GetY() < averageMotorSpeed && -JoyAccel.GetY() < -0.1) {
      accelerationSpeed = accelStartSpeed - deltaSpeed;
    } else {
      accelerationSpeed = -JoyAccel1.GetY();
      
    }
  }
  changeInY = -JoyAccel1.GetY() - averageMotorSpeed;
}

void Robot::TeleopPeriodic() {
  //Joysticks
  if (inverted){
    JoyY = JoyAccel1.GetY();
  } else {
    JoyY = -JoyAccel1.GetY();
  }
  WheelX = RaceWheel.GetX();

  accelerate(0.1);

  //Drive Code
  //Button 5 on the wheel activates point turning
  if (RaceWheel.GetRawButton(5)) {
    LeftMotorsSpeed(WheelX);
    RightMotorsSpeed(-WheelX);
  } 
   //Regular Turning
  else if((WheelX < -0.01 || WheelX > 0.01) && (JoyY > 0.06 || JoyY < -0.06)){
    //LeftMotorsSpeed(JoyY + WheelX);
    //RightMotorsSpeed(JoyY - WheelX);
  }
  //Code for driving straight  
  else if (JoyY > 0.1 || JoyY < -0.1 ){
    //LeftMotorsSpeed(JoyY);                 
    //RightMotorsSpeed(JoyY);
  } 
  //Code for if nothing is pressed
  else {
    LeftMotorsSpeed(0);
    RightMotorsSpeed(0);
  }
  //Inverts values
  if (JoyAccel1.GetRawButtonPressed(1)){
    inverted = !inverted;
  }

  //Putting values into Shuffleboard
  frc::SmartDashboard::PutNumber("Gyro Angle", gyro.GetAngle());
  //Get encoder values from falcons (built in encoders)
  frc::SmartDashboard::PutNumber("RightEncoderOne", RightMotorOne.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("LeftEncoderOne", LeftMotorOne.GetSelectedSensorPosition());
  //Get the current speed of each side
  frc::SmartDashboard::PutNumber("RightMotorSpeed", RightMotorOne.GetMotorOutputPercent());
  frc::SmartDashboard::PutNumber("LeftMotorsSpeed", LeftMotorOne.GetMotorOutputPercent());
  frc::SmartDashboard::PutNumber("AccelerationSpeed", accelerationSpeed);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif