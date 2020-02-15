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
//Color Sensor
#include <frc/util/color.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>

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
frc::ADXRS450_Gyro *gyro;

//Joysticks
frc::Joystick JoyAccel1{0}, Xbox{1}, RaceWheel{2};

double JoyY;
double WheelX;
int currentAutoStep;
bool inverted = false;
bool isDelayTimeStampSet;
double delayTimeStamp;
double accelerationRate = 0.5;

//Acceleration Variables
double accelerationSpeed;
double accelTimeStamp;
double changeInY;
double accelStartSpeed;
double deltaSpeed;
bool isAccelTimeStampSet;
bool wasInverted;

//Wheel Diameter: between 6.1 and 6.2 (6.15)
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

//Color Sensor
static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
rev::ColorSensorV3 m_colorSensor{i2cPort};
rev::ColorMatch m_colorMatcher;
//Colors (RGB values [0-1])
static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  srx.Set(ControlMode::PercentOutput, 0);

  gyro = new frc::ADXRS450_Gyro();
  gyro->Reset();
  
  //Color Sensor
  m_colorMatcher.AddColorMatch(kBlueTarget);
  m_colorMatcher.AddColorMatch(kGreenTarget);
  m_colorMatcher.AddColorMatch(kRedTarget);
  m_colorMatcher.AddColorMatch(kYellowTarget);

  //Gyro Setup
  //gyro.InitGyro();
  //gyro.Calibrate();
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

void rotationalAcceleration() {
  /*
  gyro->GetAngle();
  gyro->GetRate();

  What we know:
  Radius of Wheel: 3 in
  Rotation Rate: gyro->GetRate()
  Motor Speed: ?

  What we want to do:
  Accelerate turning based on rotations of the wheel rather than percent power

  Attempt:
  double amountToAccelerate = sqrt(gyro->GetRate());
  double motorAccelerationSpeed = amountToAccelerate (angular velocity) * 3 (3 in radius);


  rotationalAcceleration
  LeftMotorsSpeed(motorAccelerationSpeed);
  */
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
      frc::SmartDashboard::PutNumber("Gyro Rate", gyro->GetRate());
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
void accelerate(double percentPerSecond, double yInput){
  double averageMotorSpeed = (-(LeftMotorOne.GetMotorOutputPercent()) + RightMotorOne.GetMotorOutputPercent())/2;
  if (!isAccelTimeStampSet || ((changeInY > 0 && yInput - averageMotorSpeed < 0) || (changeInY < 0 && yInput - averageMotorSpeed > 0))) {
    isAccelTimeStampSet = true;
    accelTimeStamp = frc::Timer::GetFPGATimestamp();
    accelStartSpeed = yInput;
    if ((wasInverted && inverted) || (!wasInverted && !inverted)){
      accelStartSpeed = yInput;
    } else if ((!wasInverted && inverted) || (wasInverted && !inverted)) {
      accelStartSpeed = 0;
    }
  } else {
    deltaSpeed = (frc::Timer::GetFPGATimestamp() - accelTimeStamp) * percentPerSecond;
    if (yInput > averageMotorSpeed && yInput > 0.05) {
      accelerationSpeed = accelStartSpeed + deltaSpeed;
    } else if (yInput < averageMotorSpeed && yInput < -0.05) {
      accelerationSpeed = accelStartSpeed - deltaSpeed;
    } else if (yInput > -0.05 && yInput < 0.05){
      isAccelTimeStampSet = false;
      accelerationSpeed = 0;
      accelTimeStamp = 0;
      changeInY = 0;
      accelStartSpeed = 0;
      deltaSpeed = 0;
    } else {
      accelerationSpeed = yInput;
    }
  }
  changeInY = yInput - averageMotorSpeed;
  wasInverted = inverted;
}

void Robot::TeleopPeriodic() {
  //Color Sensor Code
  frc::Color detectedColor = m_colorSensor.GetColor();
  std::string colorString;
  double confidence = 0.0;
  frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

  if (matchedColor == kBlueTarget) {
    colorString = "Blue";
  }
  else if(matchedColor == kRedTarget) {
    colorString = "Red";
  }
  else if(matchedColor == kGreenTarget) {
    colorString = "Green";
  }
  else if(matchedColor == kYellowTarget) {
    colorString = "Yellow";
  }
  else {
    colorString = "Unknown";
  }
  //Put color values in shuffleboard
  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
  frc::SmartDashboard::PutNumber("Confidence", confidence);
  frc::SmartDashboard::PutString("Detected Color", colorString);
  
  //Joysticks
  if (inverted){
    JoyY = JoyAccel1.GetY();
  } else {
    JoyY = -JoyAccel1.GetY();
  }
  WheelX = RaceWheel.GetX();

  if (RaceWheel.GetRawButtonPressed(8) && accelerationRate < 0.95) {
    accelerationRate = accelerationRate + 0.1;
  } else if (RaceWheel.GetRawButtonPressed(12) && accelerationRate > 0.1) {
    accelerationRate = accelerationRate - 0.1;
  }
  
  accelerate(accelerationRate, JoyY);

  //Drive Code
  //Button 5 on the wheel activates point turning
  if (RaceWheel.GetRawButton(5)) {
    LeftMotorsSpeed(WheelX);
    RightMotorsSpeed(-WheelX);
  } 
   //Regular Turning
  else if((WheelX < -0.05 || WheelX > 0.05) && (JoyY > 0.05 || JoyY < -0.05)){
    LeftMotorsSpeed(accelerationSpeed + fabs(accelerationSpeed) * WheelX);
    RightMotorsSpeed(accelerationSpeed - fabs(accelerationSpeed) * WheelX);
  }
  //Code for driving straight  
  else if (JoyY > 0.05 || JoyY < -0.05){
    LeftMotorsSpeed(accelerationSpeed);                 
    RightMotorsSpeed(accelerationSpeed);
  } 
  //Code for if nothing is pressed
  else {
    LeftMotorsSpeed(0);
    RightMotorsSpeed(0);
  }
  //Inverts values
  if (JoyAccel1.GetRawButtonPressed(1)){
    inverted = !inverted;
    //Resets Acceleration
    //accelStartSpeed = 0;
  }

  //Putting values into Shuffleboard
  frc::SmartDashboard::PutNumber("Gyro Angle", gyro->GetAngle());
  frc::SmartDashboard::PutNumber("Gyro Rate", gyro->GetRate());
  //Get encoder values from falcons (built in encoders)
  frc::SmartDashboard::PutNumber("RightEncoderOne", RightMotorOne.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("LeftEncoderOne", LeftMotorOne.GetSelectedSensorPosition());
  //Get the current speed of each side
  frc::SmartDashboard::PutNumber("RightMotorSpeed", RightMotorOne.GetMotorOutputPercent());
  frc::SmartDashboard::PutNumber("LeftMotorsSpeed", LeftMotorOne.GetMotorOutputPercent());
  frc::SmartDashboard::PutNumber("AccelerationSpeed", accelerationSpeed);
  frc::SmartDashboard::PutNumber("Acceleration Rate", accelerationRate);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif