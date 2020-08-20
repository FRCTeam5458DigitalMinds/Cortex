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
#include <frc/Solenoid.h>
//Color Sensor
#include <frc/util/color.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
using namespace std; 

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

//Color Sensor
TalonSRX colorMotor{5};

//Gyro
frc::ADXRS450_Gyro *gyro;

//Intakes
TalonSRX FrontIntake{100};
TalonSRX BackIntake{101};

//Conveyor
TalonSRX LeftConveyor{102};
TalonSRX RightConveyor{103};

//Shooter
TalonFX LeftShooter{104};
TalonFX RightShooter{105};

/*
Xbox Buttons:
Right Bumper : 6
Left Bumper: 5
Right Trigger: Axis 3
Left Trigger: Axis 2
D-Pad Up: POV Up
D-Pad Down: POV Down
A: 1
B: 2
X: 3
Y: 4
*/
//Joysticks
frc::Joystick JoyAccel1{0}, Xbox{1}, RaceWheel{2};

//Solenoids
frc::Solenoid solenoid0{0}; 
frc::Solenoid ConveyorPiston{1}; 
frc::Solenoid BackIntakePiston{2}; 
frc::Solenoid FrontIntakePiston{3}; 
frc::Solenoid solenoid4{4}; 

//Teleop variables
double JoyY;
double WheelX;
int currentAutoStep;
bool inverted = false;
bool isDelayTimeStampSet;
double delayTimeStamp;
double accelerationRate = 0.5;
bool switchedIntakes;

//Auto Variables
//Turning
double turnTimeStamp;
double autoTimeStamp;
double turnSpeed;
int turnStep;
double highestTurnSpeed;
double turnAccel;
//Distance
int distanceStep; //Current step of the goDistance function
double distanceTimeStamp; //Used to keep track of how many seconds have passed since a certain point
double someDistance; // The distance the bot reached when it went half the encoder units
double someSpeed; //The speed the bot was traveling when it started to slow down
//Arc Turning
int arcStep;
double arcTimeStamp;
double highestArcSpeed;

//Other
bool isAutoRunning;
double autoStartingAngle;
double averageEncoderValue;
double averageMotorSpeed;
double motorAccelerationSpeed;
double amountToAccelerate;

//Acceleration Variables (Teleop)
double accelerationSpeed;
double accelTimeStamp;
double changeInY;
double accelStartSpeed;
double deltaSpeed;
bool isAccelTimeStampSet;
bool wasInverted;

//Gyro Variables
double gyroFact;
double turnFact;
double lastSumAngle;
double correctionAngle;
bool isStartingAngleSet;
double startingAngle;
double someAngle;

//TEST
float lastSumAngle2 = 0, lastSumDistance2 = 0;
double correctionAngle2 = 0, correctionDistance2 = 0;

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
void Shooter(double speed){
  LeftShooter.Set(ControlMode::PercentOutput, speed);
  RightShooter.Set(ControlMode::PercentOutput, -speed);
}
void Conveyor(double leftSpeed, double rightSpeed){
  LeftConveyor.Set(ControlMode::PercentOutput, leftSpeed);
  RightConveyor.Set(ControlMode::PercentOutput, rightSpeed);
}


void music() {
  /*If we ever have extra time,
  we can make music with falcons 
  http://www.ctr-electronics.com/downloads/api/cpp/html/classctre_1_1phoenix_1_1music_1_1_orchestra.html#a5e2089c7e3c582566f5da6ffa4d79787
  */
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

  //Initialize Gyro
  gyro = new frc::ADXRS450_Gyro();
  gyro->Reset();
  startingAngle = 0;
  
  //Color Sensor
  m_colorMatcher.AddColorMatch(kBlueTarget);
  m_colorMatcher.AddColorMatch(kGreenTarget);
  m_colorMatcher.AddColorMatch(kRedTarget);
  m_colorMatcher.AddColorMatch(kYellowTarget);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  //Values we want to see whenever the bot is running
  averageMotorSpeed = (-(LeftMotorOne.GetMotorOutputPercent()) + RightMotorOne.GetMotorOutputPercent())/2;
  averageEncoderValue = (-(LeftMotorOne.GetSelectedSensorPosition()) + RightMotorOne.GetSelectedSensorPosition())/2;

  //Put values into shuffleboard
  frc::SmartDashboard::PutNumber("Gyro Angle", gyro->GetAngle());
  //frc::SmartDashboard::PutNumber("Gyro Rate", gyro->GetRate());
  frc::SmartDashboard::PutNumber("Correction Angle", correctionAngle2);
  frc::SmartDashboard::PutNumber("Correction Distance", correctionDistance2);
  frc::SmartDashboard::PutNumber("Some Angle", someAngle);
  //frc::SmartDashboard::PutNumber("Left Motor Output", LeftMotorOne.GetMotorOutputPercent());
  //frc::SmartDashboard::PutNumber("Right Motor Output", RightMotorOne.GetMotorOutputPercent());
  frc::SmartDashboard::PutNumber("Turn Step", turnStep);
  //frc::SmartDashboard::PutNumber("Highest Turn Speed", highestTurnSpeed);
  frc::SmartDashboard::PutNumber("CurrentAutoStep", currentAutoStep);
  frc::SmartDashboard::PutNumber("Starting Angle", startingAngle);
  //frc::SmartDashboard::PutNumber("Distance Step", distanceStep);
  frc::SmartDashboard::PutNumber("Average Encoder Value", averageEncoderValue);
  //frc::SmartDashboard::PutNumber("Some Distance", someDistance);
  //frc::SmartDashboard::PutNumber("Some Speed", someSpeed);
  frc::SmartDashboard::PutNumber("Xbox POV", Xbox.GetPOV());

  /*
  Now that we're using drivingCorrection() function during teleop and auto, we shouldn't need this
  //Correction angle
  if (gyro->GetRate() > 0) {
    correctionAngle = gyro->GetAngle()/180;
  } else if (gyro->GetRate() < 0) {
    correctionAngle = gyro->GetAngle()/180;
  } 
  */
}

void drivingCorrection(){
  if (!isStartingAngleSet) {
    if (isAutoRunning){
      startingAngle = autoStartingAngle;
    } else {
      startingAngle = gyro->GetAngle();
    }
    isStartingAngleSet = true;
  } else {
    //Correction angle
    correctionAngle = (gyro->GetAngle() - startingAngle)/45;
  } 
}

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
    someAngle = 0;
    highestTurnSpeed = 0;
    turnStep = 1;
    distanceStep = 1;
    arcStep = 1;
    LeftMotorOne.SetSelectedSensorPosition(0);
    RightMotorOne.SetSelectedSensorPosition(0);

    gyro->Calibrate();

    //Reset correction variables
    isAutoRunning = true;
    isStartingAngleSet = false;
    startingAngle = 0;
    autoStartingAngle = gyro->GetAngle();
    
    FrontIntakePiston.Set(false);
    BackIntakePiston.Set(false);
    ConveyorPiston.Set(false);
  }
}

//Autonomous Functions
void goDistance(double inches, double percentPerSecond, double maxSpeed) {

  /*
  double encoderUnits = inches * 4000/12;
  double averageEncoderValue = (-(LeftMotorOne.GetSelectedSensorPosition()) + RightMotorOne.GetSelectedSensorPosition())/2;
  //double distanceLeft = encoderUnits - ((LeftMotorOne.GetSelectedSensorPosition() + RightMotorOne.GetSelectedSensorPosition()) / 2)
  if (averageEncoderValue < encoderUnits && encoderUnits > 0) {
    LeftMotorsSpeed(speed - (fabs(speed) * correctionAngle));                 
    RightMotorsSpeed(speed + (fabs(speed) * correctionAngle));
  }
  else if (averageEncoderValue > encoderUnits && encoderUnits < 0) {
    LeftMotorsSpeed(-speed - (fabs(speed) * correctionAngle));                 
    RightMotorsSpeed(-speed + (fabs(speed) * correctionAngle));
  }
  else {
    isStartingAngleSet = false;
    startingAngle = 0;
    LeftMotorOne.SetSelectedSensorPosition(0);
    RightMotorOne.SetSelectedSensorPosition(0);
    currentAutoStep = currentAutoStep + 1;
  } */

  // float lastSumAngle = 0, lastSumDistance = 0;
  // double pgain = frc::SmartDashboard::GetNumber("pgain", .1);
  // double dgain = frc::SmartDashboard::GetNumber("dgain", .05);

  // averageEncoderValue = (-(LeftMotorOne.GetSelectedSensorPosition() + RightMotorOne.GetSelectedSensorPosition()))/2;
  // float sumDistance = averageEncoderValue;
  // float derivDistance = sumDistance - lastSumDistance2;
  // correctionDistance2 = ((inches - sumDistance) * .06) - (derivDistance * .4);

  // float sumAngle = gyro->GetAngle();
  // float derivAngle = sumAngle - lastSumAngle2;
  // correctionAngle2 = (sumAngle * pgain) + (derivAngle * dgain); //kpe ,008
    

  // LeftMotorsSpeed(-correctionAngle2);
  // RightMotorsSpeed(correctionAngle2);

  // lastSumAngle2 = sumAngle;
  // lastSumDistance2 = sumDistance;
  
  double encoderUnits = inches * 6075/12;
  double accelSpeed = (frc::Timer::GetFPGATimestamp() - distanceTimeStamp) * percentPerSecond;

  frc::SmartDashboard::PutNumber("encoderUnits", encoderUnits);

  drivingCorrection();

  switch (distanceStep) {
    case 1:
    LeftMotorOne.SetSelectedSensorPosition(0);
    RightMotorOne.SetSelectedSensorPosition(0);
    distanceTimeStamp = frc::Timer::GetFPGATimestamp();
    distanceStep += 1;
    break;

    case 2:
    if (encoderUnits > 0){
      LeftMotorsSpeed(accelSpeed - (fabs(accelSpeed) * correctionAngle));
      RightMotorsSpeed(accelSpeed + (fabs(accelSpeed) * correctionAngle));
    } else if (encoderUnits < 0){
      LeftMotorsSpeed(-accelSpeed - (fabs(accelSpeed) * correctionAngle));
      RightMotorsSpeed(-accelSpeed + (fabs(accelSpeed) * correctionAngle));
    }
    if (fabs(averageMotorSpeed) >= maxSpeed) {
      someDistance = averageEncoderValue;
      someSpeed = maxSpeed;
      distanceStep += 1;
    } else if (fabs(averageEncoderValue) > fabs(encoderUnits)/2) {
      someSpeed = fabs(averageMotorSpeed);
      distanceTimeStamp = frc::Timer::GetFPGATimestamp();
      distanceStep += 2;
    }
    break;

    case 3:
    if (encoderUnits > 0){
      LeftMotorsSpeed(maxSpeed - (fabs(maxSpeed) * correctionAngle));
      RightMotorsSpeed(maxSpeed + (fabs(maxSpeed) * correctionAngle));
    } else if (encoderUnits < 0){
      LeftMotorsSpeed(-maxSpeed + (fabs(maxSpeed) * correctionAngle));
      RightMotorsSpeed(-maxSpeed - (fabs(maxSpeed) * correctionAngle));
    }
    if (fabs(averageEncoderValue) > fabs(encoderUnits) - fabs(someDistance)) {
      someSpeed = fabs(averageMotorSpeed);
      distanceTimeStamp = frc::Timer::GetFPGATimestamp();
      distanceStep += 1;
    }
    break;
    //We could change step for to be like void turn by having it deccelerate based on someSpeed / maxSpeed
    case 4:
    if (encoderUnits > 0) {
      LeftMotorsSpeed((someSpeed - accelSpeed * (someSpeed / maxSpeed)) - (fabs(someSpeed - accelSpeed) * correctionAngle));
      RightMotorsSpeed((someSpeed - accelSpeed * (someSpeed / maxSpeed)) + (fabs(someSpeed - accelSpeed) * correctionAngle));
    } else if (encoderUnits < 0) {
      LeftMotorsSpeed((someSpeed + accelSpeed * (someSpeed / maxSpeed)) - (fabs(someSpeed + accelSpeed) * correctionAngle));
      RightMotorsSpeed((someSpeed + accelSpeed * (someSpeed/maxSpeed)) + (fabs(someSpeed + accelSpeed) * correctionAngle));
    }
    if (fabs(averageEncoderValue) >= fabs(encoderUnits) || averageMotorSpeed == 0) {
      distanceStep += 1;
    }
    break;

    case 5:
    LeftMotorsSpeed(0);
    RightMotorsSpeed(0);
    isStartingAngleSet = false;
    currentAutoStep += 1;
    distanceStep = 1;
    break;

    default:
    LeftMotorsSpeed(0);
    RightMotorsSpeed(0);
  }
}

void turn(double degrees, double percentPerSecond, double maxSpeed) {
  
  averageMotorSpeed = (LeftMotorOne.GetMotorOutputPercent() + RightMotorOne.GetMotorOutputPercent())/2;

  /*
  Brainstorming:
  Accelerate in degrees per second rather than percent per second.
  Use gyro->GetRate() to know if we have reached the maximum speed we want to go
  Increase rate until gyro->GetRate() equals a certain value
  
  
  
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
  */
/*
//velocity = angular velocity (deg/sec) * radius (in);
amountToAccelerate = sqrt(gyro->GetRate());
//Motor Accel Speed in inches/sec
motorAccelerationSpeed = (amountToAccelerate * 3 * 6.28)/360;
turnAccel = (frc::Timer::GetFPGATimestamp() - autoTimeStamp) * motorAccelerationSpeed;

//If motorAccelSpeed is positive, add the change in acceleration to it and set the motors to that value
 if (motorAccelerationSpeed > 0) {
   turnSpeed = motorAccelerationSpeed + turnAccel;
   LeftMotorsSpeed(turnSpeed);
   RightMotorsSpeed(-turnSpeed);
 } else if (motorAccelerationSpeed < 0) {
   turnSpeed = motorAccelerationSpeed - turnAccel;
   LeftMotorsSpeed(-turnSpeed);
   RightMotorsSpeed(turnSpeed);
 } else {
   currentAutoStep = currentAutoStep + 1;
 }
*/
/*
  double averageMotorSpeed = (LeftMotorOne.GetMotorOutputPercent() + RightMotorOne.GetMotorOutputPercent())/2;
  if (!isTurnTimeStampSet){
    turnTimeStamp = frc::Timer::GetFPGATimestamp();
    isTurnTimeStampSet = true;
  } else {
    if (fabs(averageMotorSpeed) < maxSpeed && fabs(gyro->GetAngle()) < fabs(degrees/4)) {
      if (gyro->GetAngle() < degrees){
        LeftMotorsSpeed((frc::Timer::GetFPGATimestamp() - turnTimeStamp) * percentPerSecond);
        RightMotorsSpeed(-(frc::Timer::GetFPGATimestamp() - turnTimeStamp) * percentPerSecond); 
      } else if (gyro->GetAngle() > degrees) {
        LeftMotorsSpeed((frc::Timer::GetFPGATimestamp() - turnTimeStamp) * percentPerSecond);
        RightMotorsSpeed(-(frc::Timer::GetFPGATimestamp() - turnTimeStamp) * percentPerSecond);
      }
    } else if (someAngle == 0) {
      someAngle = gyro->GetAngle();
    } else if (fabs(gyro->GetAngle()) < fabs(degrees) - fabs(someAngle)) {
      if (gyro->GetAngle() < degrees){
        LeftMotorsSpeed(maxSpeed);
        RightMotorsSpeed(-maxSpeed);
      } else if (gyro->GetAngle() > degrees){
        LeftMotorsSpeed(-maxSpeed);
        RightMotorsSpeed(maxSpeed);
      } 
      turnTimeStamp = frc::Timer::GetFPGATimestamp();
    }
    else if (fabs(gyro->GetAngle()) > fabs(degrees) - fabs(someAngle) && fabs(averageMotorSpeed) > 0 && fabs(gyro->GetAngle()) < degrees) {
      if (gyro->GetAngle() < degrees) {
        LeftMotorsSpeed(maxSpeed - (frc::Timer::GetFPGATimestamp() - turnTimeStamp) * percentPerSecond);
        RightMotorsSpeed(-(maxSpeed - (frc::Timer::GetFPGATimestamp() - turnTimeStamp) * percentPerSecond));
      } else if (gyro->GetAngle() > degrees) {
        LeftMotorsSpeed(-(maxSpeed - (frc::Timer::GetFPGATimestamp() - turnTimeStamp) * percentPerSecond));
  
      }      RightMotorsSpeed(maxSpeed - (frc::Timer::GetFPGATimestamp()- turnTimeStamp) * percentPerSecond);
    } else {
      LeftMotorsSpeed(0);
      RightMotorsSpeed(0);
      someAngle = 0;
      isTurnTimeStampSet = false;
      currentAutoStep = currentAutoStep + 1;
    }
  }
*/
  switch (turnStep){
    case 1:
    gyro->Reset();
    turnTimeStamp = frc::Timer::GetFPGATimestamp();
    turnStep += 1;
    break;

    case 2:
    if (gyro->GetAngle() < degrees) {
      LeftMotorsSpeed((frc::Timer::GetFPGATimestamp() - turnTimeStamp) * percentPerSecond);
      RightMotorsSpeed(-(frc::Timer::GetFPGATimestamp() - turnTimeStamp) * percentPerSecond); 
    } else if (gyro->GetAngle() > degrees) {
      LeftMotorsSpeed((frc::Timer::GetFPGATimestamp() - turnTimeStamp) * percentPerSecond);
      RightMotorsSpeed(-(frc::Timer::GetFPGATimestamp() - turnTimeStamp) * percentPerSecond);
    }
    if (fabs(averageMotorSpeed) >= maxSpeed) {
      highestTurnSpeed = maxSpeed;
      someAngle = gyro->GetAngle();
      turnStep += 1;
    } else if (fabs(gyro->GetAngle()) > fabs(degrees/2)){
      highestTurnSpeed = fabs(averageMotorSpeed);
      someAngle = gyro->GetAngle();
      turnTimeStamp = frc::Timer::GetFPGATimestamp();
      turnStep += 2;
    }
    break;

    case 3:
    if (gyro->GetAngle() < degrees) {
      LeftMotorsSpeed(highestTurnSpeed);
      RightMotorsSpeed(-highestTurnSpeed);
    } else if (gyro->GetAngle() > degrees) {
      LeftMotorsSpeed(-highestTurnSpeed);
      RightMotorsSpeed(highestTurnSpeed);
    }
    if ((fabs(gyro->GetAngle()) > fabs(degrees) - fabs(someAngle))) {
      turnTimeStamp = frc::Timer::GetFPGATimestamp();
      turnStep += 1;
    }
    break;

    case 4:
    if (gyro->GetAngle() < degrees) {
      LeftMotorsSpeed(highestTurnSpeed - ((frc::Timer::GetFPGATimestamp() - turnTimeStamp) * (percentPerSecond * highestTurnSpeed / maxSpeed)));
      RightMotorsSpeed(-(highestTurnSpeed - ((frc::Timer::GetFPGATimestamp() - turnTimeStamp) * (percentPerSecond * highestTurnSpeed / maxSpeed))));
    } else if (gyro->GetAngle() > degrees) {
      LeftMotorsSpeed(-(highestTurnSpeed - ((frc::Timer::GetFPGATimestamp() - turnTimeStamp) * (percentPerSecond * highestTurnSpeed / maxSpeed))));
      RightMotorsSpeed(highestTurnSpeed - ((frc::Timer::GetFPGATimestamp() - turnTimeStamp) * (percentPerSecond * highestTurnSpeed / maxSpeed)));
    }
    if (fabs(gyro->GetAngle()) >= fabs(degrees) || averageMotorSpeed == 0){
      turnStep += 1;
    }
    break;
    
    case 5:
    LeftMotorsSpeed(0);
    RightMotorsSpeed(0);
    isStartingAngleSet = false;
    autoStartingAngle = degrees;
    currentAutoStep += 1;
    turnStep = 1;
    break;

    default:
    LeftMotorsSpeed(0);
    RightMotorsSpeed(0);
  }
  frc::SmartDashboard::PutNumber("Degrees", degrees);
  frc::SmartDashboard::PutNumber("Deceleration Rate", ((fabs(degrees) - fabs(gyro->GetAngle()) / fabs(someAngle))));
}

void arcTurning(double degrees, double radius, double percentPerSecond, double maxSpeed, bool isBackwards) {
  double centerDist;
  double leftDist;
  double rightDist;

  switch (arcStep) {
    case 1:
    gyro->Reset();
    arcTimeStamp = frc::Timer::GetFPGATimestamp();
    centerDist = 2 * 3.1415 * radius * (fabs(degrees)/360);
    if ((degrees > 0 && !isBackwards) || (degrees < 0 && isBackwards)) {
      leftDist = 2 * 3.1415 * (radius + 13.75) * (fabs(degrees)/360);
      rightDist = 2 * 3.1415 * (radius - 13.75) * (fabs(degrees)/360);
    } else if ((degrees < 0 && !isBackwards || (degrees > 0 && isBackwards))) {
      leftDist = 2 * 3.1415 * (radius - 13.75) * (fabs(degrees)/360);
      rightDist = 2 * 3.1415 * (radius + 13.75) * (fabs(degrees)/360);
    }
    arcStep += 1;
    break;

    case 2:
    if (isBackwards){
      LeftMotorsSpeed(-(frc::Timer::GetFPGATimestamp() - arcTimeStamp) * percentPerSecond * (leftDist / centerDist));
      RightMotorsSpeed(-(frc::Timer::GetFPGATimestamp() - arcTimeStamp) * percentPerSecond * (rightDist / centerDist));
    } else if (!isBackwards) {
      LeftMotorsSpeed((frc::Timer::GetFPGATimestamp() - arcTimeStamp) * percentPerSecond * (leftDist / centerDist));
      RightMotorsSpeed((frc::Timer::GetFPGATimestamp() - arcTimeStamp) * percentPerSecond * (rightDist / centerDist));
    }

    if (fabs(averageMotorSpeed) >= maxSpeed) {
      someAngle = gyro->GetAngle();
      highestArcSpeed = maxSpeed;
      arcStep += 1;
    } else if (fabs(gyro->GetAngle()) > fabs(degrees) / 2) {
      highestArcSpeed = averageMotorSpeed;
      someAngle = gyro->GetAngle();
      arcTimeStamp = frc::Timer::GetFPGATimestamp();
      arcStep += 2;
    }
    break;

    case 3:
    if (isBackwards){
      LeftMotorsSpeed(-maxSpeed * (leftDist / centerDist));
      RightMotorsSpeed(-maxSpeed * (rightDist / centerDist));
    } else if (!isBackwards) {
      LeftMotorsSpeed(maxSpeed * (leftDist / centerDist));
      RightMotorsSpeed(maxSpeed * (rightDist / centerDist));
    }
    if (fabs(gyro->GetAngle() > fabs(degrees) - someAngle)) {
      arcTimeStamp = frc::Timer::GetFPGATimestamp();
      arcStep += 1;
    }
    break;

    case 4:
    if (isBackwards) {
      LeftMotorsSpeed(-highestArcSpeed + ((frc::Timer::GetFPGATimestamp() - arcTimeStamp) * percentPerSecond * (leftDist / centerDist) * (highestArcSpeed / maxSpeed)));
      RightMotorsSpeed(-highestArcSpeed + ((frc::Timer::GetFPGATimestamp() - arcTimeStamp) * percentPerSecond * (rightDist / centerDist) * (highestArcSpeed / maxSpeed)));
    } else if (!isBackwards) {
      LeftMotorsSpeed(highestArcSpeed - ((frc::Timer::GetFPGATimestamp() - arcTimeStamp) * percentPerSecond * (leftDist / centerDist) * (highestArcSpeed / maxSpeed)));
      RightMotorsSpeed(highestArcSpeed - ((frc::Timer::GetFPGATimestamp() - arcTimeStamp) * percentPerSecond * (rightDist / centerDist) * (highestArcSpeed / maxSpeed)));
    }
    if (averageMotorSpeed == 0 || fabs(gyro->GetAngle()) >= fabs(degrees)){
      arcStep += 1;
    }
    break;

    case 5:
    LeftMotorsSpeed(0);
    RightMotorsSpeed(0);
    isStartingAngleSet = false;
    autoStartingAngle = degrees;
    arcStep = 1;
    currentAutoStep += 1;
    break;
    
    default:
    LeftMotorsSpeed(0);
    RightMotorsSpeed(0);
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
    // Gyro Correction variables
    float sumAngle = gyro->GetAngle();
		float derivAngle = sumAngle - lastSumAngle;

    switch (currentAutoStep){
      case 1:
      arcTurning(180, 60, 0.1, 0.2, false);
      break;
      
      case 2:
      delay(1);
      break;

      case 3:
      arcTurning(180, 60, 0.1, 0.2, true);      
      break;

      case 4:
      stopAll();
      isAutoRunning = false;
      break;

      default:
      LeftMotorsSpeed(0);
      RightMotorsSpeed(0);
      isAutoRunning = false;
    }
  }
}

void TestInit(){
    //Auto Init
    currentAutoStep = 1;
    isDelayTimeStampSet = false;
    delayTimeStamp = 0;
    someAngle = 0;
    highestTurnSpeed = 0;
    turnStep = 1;
    distanceStep = 1;
    LeftMotorOne.SetSelectedSensorPosition(0);
    RightMotorOne.SetSelectedSensorPosition(0);
    gyro->Calibrate();

    //Reset correction variables
    isAutoRunning = true;
    isStartingAngleSet = false;
    startingAngle = 0;
    autoStartingAngle = gyro->GetAngle();
}

void Robot::TestPeriodic(){
  goDistance(0,0,0);
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
  accelerationRate = 0.5;
  gyro->Reset();
  isAutoRunning = false;
  switchedIntakes = false;

  FrontIntakePiston.Set(false);
  BackIntakePiston.Set(false);
  ConveyorPiston.Set(false);
}

//Teleop Functions
void accelerate(double percentPerSecond, double yInput){
  averageMotorSpeed = (-(LeftMotorOne.GetMotorOutputPercent()) + RightMotorOne.GetMotorOutputPercent())/2;

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
  //Gyro Correction variables
  float sumAngle = gyro->GetAngle();
	float derivAngle = sumAngle - lastSumAngle;

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
  
  //Runs accelerate function during periodic
  accelerate(accelerationRate, JoyY);

  //Sets startingAngle back to false
  if ((JoyY < 0.05 && JoyY > -0.05) || (WheelX > 0.05 || WheelX < -0.05)) {
    isStartingAngleSet = false;
  }
  
  //Drive Code
  //Button 5 on the wheel activates point turning
  if (RaceWheel.GetRawButton(5)) {
    if (WheelX > 0) {
      LeftMotorsSpeed(WheelX * WheelX);
      RightMotorsSpeed(-(WheelX * WheelX));
    } else if (WheelX < 0) {
      LeftMotorsSpeed(-(WheelX * WheelX));
      RightMotorsSpeed(WheelX * WheelX);
    }
  } 
  //Regular Turning
  else if((WheelX < -0.05 || WheelX > 0.05) && (JoyY > 0.05 || JoyY < -0.05)){
    LeftMotorsSpeed(accelerationSpeed + (fabs(accelerationSpeed) * WheelX));
    RightMotorsSpeed(accelerationSpeed - (fabs(accelerationSpeed) * WheelX));
  }
  //Code for driving straight  
  else if (JoyY > 0.05 || JoyY < -0.05) {
    // Runs correction function during periodic
    drivingCorrection();

    LeftMotorsSpeed(accelerationSpeed - (fabs(accelerationSpeed)* correctionAngle));                 
    RightMotorsSpeed(accelerationSpeed + (fabs(accelerationSpeed) * correctionAngle));
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

  //lastSumAngle = sumAngle;

  //Code for shooting
  if (Xbox.GetRawButtonPressed(3)) {
    Shooter(0.5); //if we increase shooter speed, remember to also increase speed average motor speed has to be greater than
    ConveyorPiston.Set(!ConveyorPiston.Get());
    if ((fabs(LeftShooter.GetMotorOutputPercent()) + fabs(RightShooter.GetMotorOutputPercent()) / 2) > 0.45) {
      Conveyor(-0.2, -0.2);
      ConveyorPiston.Set(true);
    }
  } else {
    Shooter(0);
    Conveyor(0, 0);
    ConveyorPiston.Set(false);
  }

  //Code for intaking
  if (Xbox.GetRawButton(4)){
    FrontIntake.Set(ControlMode::PercentOutput, 0.75);
    BackIntake.Set(ControlMode::PercentOutput, 0);
    Conveyor(-0.2, -0.2);
  } else if (Xbox.GetRawButton(1)) {
    FrontIntake.Set(ControlMode::PercentOutput, 0);
    BackIntake.Set(ControlMode::PercentOutput, 0.75);
    Conveyor(-0.2, -0.2);
  }

  //Code for spitting
  if (Xbox.GetPOV() == 0){
    FrontIntake.Set(ControlMode::PercentOutput, -0.75);
    BackIntake.Set(ControlMode::PercentOutput, 0);
    Conveyor(-0.2, 0.2);
  } else if (Xbox.GetPOV() == 180){
    FrontIntake.Set(ControlMode::PercentOutput, 0);
    BackIntake.Set(ControlMode::PercentOutput, -0.75);
    Conveyor(0.2, -0.2);
  } else {
    FrontIntake.Set(ControlMode::PercentOutput, 0);
    BackIntake.Set(ControlMode::PercentOutput, 0);
    Conveyor(0, 0);
  }

  //Code for intake pistons
  if (Xbox.GetRawButtonPressed(5)) {
    BackIntakePiston.Set(!BackIntakePiston.Get());
  } 
  if (Xbox.GetRawButtonPressed(6)){
    FrontIntakePiston.Set(!FrontIntakePiston.Get());
  }
  /*if (Xbox.GetRawAxis(2) > 0.05 && !switchedIntakes){
    FrontIntakePiston.Set(!FrontIntakePiston.Get());
    BackIntakePiston.Set(!FrontIntakePiston.Get());
    switchedIntakes = true;
  } else if (Xbox.GetRawAxis(2) < 0.05){
    switchedIntakes = false;
  } */
  
  //Putting values into Shuffleboard
  //Get encoder values from falcons (built in encoders) and other motors
  frc::SmartDashboard::PutNumber("RightEncoderOne", RightMotorOne.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("LeftEncoderOne", LeftMotorOne.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Color Encoder", colorMotor.GetSelectedSensorPosition());
  //Get the current speed of each side
  frc::SmartDashboard::PutNumber("RightMotorSpeed", RightMotorOne.GetMotorOutputPercent());
  frc::SmartDashboard::PutNumber("LeftMotorsSpeed", LeftMotorOne.GetMotorOutputPercent());
  frc::SmartDashboard::PutNumber("AccelerationSpeed", accelerationSpeed);
  frc::SmartDashboard::PutNumber("Acceleration Rate", accelerationRate);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif