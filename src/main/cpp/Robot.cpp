// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  //initialize Field2d
  frc::Field2d(m_field);

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutData("Field", &m_field);

  ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};

  // set slot 0 gains
  auto& slot0Configs = talonFXConfigs.Slot0;
  slot0Configs.kS = 0.14; // Add 0.14 V output to overcome static friction
  slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
  slot0Configs.kI = 0; // no output for integrated error
  slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

  // set Motion Magic settings
  auto& motionMagicConfigs = talonFXConfigs.MotionMagic;
  motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
  motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
  motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

  //ar.GetConfigurator().Apply(talonFXConfigs);
  ar.GetConfigurator().Apply(talonFXConfigs);
  //al.SetControl(controls::Follower, 0);
  al.SetControl(ctre::phoenix6::controls::Follower{8, true});

  //config motors
  fr.ConfigFactoryDefault();
  fl.ConfigFactoryDefault();
  br.ConfigFactoryDefault();
  bl.ConfigFactoryDefault();
  
  fr.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::PulseWidthEncodedPosition, 0, 50);
  fl.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::PulseWidthEncodedPosition, 0, 50);
  br.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::PulseWidthEncodedPosition, 0, 50);
  bl.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::PulseWidthEncodedPosition, 0, 50);

  fr.SetSensorPhase(false);
  fl.SetSensorPhase(false);
  br.SetSensorPhase(false);
  bl.SetSensorPhase(false);

  fr.SetSelectedSensorPosition(0,0,10);
  fl.SetSelectedSensorPosition(0,0,10);
  br.SetSelectedSensorPosition(0,0,10);
  bl.SetSelectedSensorPosition(0,0,10);


  /*Add in the configuration for motion magic for the wheels
    _talon.SelectProfileSlot(0, 0);
    _talon.Config_kF(0, 0.3, 10);
    _talon.Config_kP(0, 0.1, 10);
    _talon.Config_kI(0, 0.0, 10);
    _talon.Config_kD(0, 0.0, 10);
    _talon.ConfigMotionCruiseVelocity(1500, 10);
    _talon.ConfigMotionAcceleration(1500, 10);
*/

  secondaryShooter.Follow(mainShooter, true);
  intakeFollow.Follow(intakeMain, false);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  m_odometry.Update(
    getRotation2d,
    frc::MecanumDriveWheelPositions{
      units::inch_t{(((fr.GetSelectedSensorPosition(0))/4096)*25)},
      units::inch_t{(((fl.GetSelectedSensorPosition(0))/4096)*25)},
      units::inch_t{(((bl.GetSelectedSensorPosition(0))/4096)*25)},
      units::inch_t{(((br.GetSelectedSensorPosition(0))/4096)*25)}
    }
  );
  m_field.SetRobotPose(m_odometry.GetPose());

  frc::SmartDashboard::PutNumber("Heading", gyro.GetAngle());
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
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

  //Create Variables for Controller Inputs
  lx = xbox.GetLeftX();
  ly = xbox.GetLeftY();
  rx = xbox.GetRightX();

  AButton = xbox.GetAButton();
  BButton = xbox.GetBButton();
  XButton = xbox.GetXButton();
  YButton = xbox.GetYButton();



  LeftTrigger = xbox.GetLeftTriggerAxis();
  LeftBumper = xbox.GetLeftBumper();
  RightTrigger = xbox.GetRightTriggerAxis();
  RightBumper = xbox.GetRightBumper();

  StartButton = xbox.GetStartButton();
  BackButton = xbox.GetBackButton();

  RightStickButton = xbox.GetRightStickButton();
  LeftStickButton = xbox.GetLeftStickButton();

  //Target Direction, Magnitude, and Turn
  direction = atan2(ly,lx);
  magnitude = hypot(lx,ly);
  turn = rx*0.7;


  //Half Trapazoidal Acceleration 

  if(magnitude>=1){
    magnitude = 1;
  }
  if(lx>topspeed){
    lx = topspeed;
  }else if(lx<-topspeed){
    lx = -topspeed;
  }
  if(ly>topspeed){
    ly = topspeed;
  }else if(ly<-topspeed){
    ly = -topspeed;
  }
  if(rx>topspeed){
    rx = topspeed;
  }else if(rx<-topspeed){
    rx = -topspeed;
  }

  spd = (spdmult);




  if(ly>=0.1 || ly<=-0.1 || lx>=0.1 ||lx<=-0.1){
    fl.Set(ControlMode::PercentOutput, flipDrive*spd*((sin(-direction+(.25*Pi)))*magnitude + flipDrive*turn));
    fr.Set(ControlMode::PercentOutput, -flipDrive*spd*((sin(-direction-(.25*Pi)))*magnitude - flipDrive*turn));
    bl.Set(ControlMode::PercentOutput, flipDrive*spd*((sin(-direction-(.25*Pi)))*magnitude + flipDrive*turn));
    br.Set(ControlMode::PercentOutput, -flipDrive*spd*((sin(-direction+(.25*Pi)))*magnitude - flipDrive*turn));
  }else if(rx>0.1 || rx<-0.1){
    fl.Set(ControlMode::PercentOutput, spd*turn);
    fr.Set(ControlMode::PercentOutput, spd*turn);
    bl.Set(ControlMode::PercentOutput, spd*turn);
    br.Set(ControlMode::PercentOutput, spd*turn);
  }else{
    direction = 0;
    fl.Set(ControlMode::PercentOutput, 0);
    fr.Set(ControlMode::PercentOutput, 0);
    bl.Set(ControlMode::PercentOutput, 0);
    br.Set(ControlMode::PercentOutput, 0);
  }


  ctre::phoenix6::controls::MotionMagicVoltage m_request{0_tr};

  if(RightBumper){

  ar.SetControl(m_request.WithPosition(39_tr));

  }
  if(BackButton){
    ar.SetControl(m_request.WithPosition(25_tr));

  }

  if(LeftBumper){
  ar.SetControl(m_request.WithPosition(1_tr));
  }


  if(XButton){
  
  ar.SetControl(m_request.WithPosition(130_tr));

  }


//Intake Statements
if(YButton){
  pickupTimer.Start();
  intakeMain.Set(-.4);
  pickupActive = 1;
}
if(pickupTimer.HasElapsed(.6_s)){
  intakeMain.Set(0);
  pickupTimer.Stop();
  pickupTimer.Reset();
  pickupActive = 0;
}

//If intaking dont shoot, Shooter statements
if(!pickupActive){
if(StartButton){
  mainShooter.Set(-.75);
  shooterDelay.Start();
}

if(shooterDelay.HasElapsed(2_s)){
  mainShooter.Set(0);
  intakeMain.Set(0);
  shooterDelay.Stop();
  shooterDelay.Reset();
}
else if(shooterDelay.HasElapsed(1_s)){
  intakeMain.Set(-.75);
}
}

//If intaking dont shoot, Shooter statements
if(!pickupActive){
if(RightStickButton){
  mainShooter.Set(-.75);
  shooterDelay.Start();
}

if(shooterDelay.HasElapsed(2_s)){
  mainShooter.Set(0);
  intakeMain.Set(0);
  shooterDelay.Stop();
  shooterDelay.Reset();
}
else if(shooterDelay.HasElapsed(1_s)){
  intakeMain.Set(-.65);
}
}

//Push Note Back out of Intake
if(!pickupActive){
if(AButton){
  pickupTimer.Start();
  intakeMain.Set(.4);
}
if(pickupTimer.HasElapsed(.05_s)){
  intakeMain.Set(0);
  pickupTimer.Stop();
  pickupTimer.Reset();
}

}


if(xbox.GetBButtonReleased()){
  flipDrive = flipDrive * -1;
}

if(LeftStickButton){
  targetAngle = atan2((4 + 19.5),(0 - 17))*(180/Pi);

}

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
