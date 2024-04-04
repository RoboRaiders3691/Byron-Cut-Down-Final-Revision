// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  //Set Up Cameras
  camera1 = frc::CameraServer::StartAutomaticCapture(0);
  camera2 = frc::CameraServer::StartAutomaticCapture(1);

  //camera1.SetPath("/dev/video0");
  //camera2.SetPath("/dev/video1");

  cameraSelection = nt::NetworkTableInstance::GetDefault().GetTable("")->GetEntry("CameraSelection");

  //initialize Field2d
  frc::Field2d m_field;

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameMid, kAutoNameMid);
  m_chooser.AddOption(kAutoNameR, kAutoNameR);
  m_chooser.AddOption(kAutoNameL, kAutoNameL);
  m_chooser.AddOption(kAutoNameL, kAutoNameS);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

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

  //Front Right Motion Magic Configs
  fr.SelectProfileSlot(0, 0);
  fr.Config_kF(0, 0.55, 10);
  fr.Config_kP(0, 0.06, 10);
  fr.Config_kI(0, 0.0, 10);
  fr.Config_kD(0, 0.6, 10);
  fr.ConfigMotionCruiseVelocity(1500, 10);
  fr.ConfigMotionAcceleration(700, 10);

  //Front Left Motion Magic Configs
  fl.SelectProfileSlot(0, 0);
  fl.Config_kF(0, 0.56, 10);
  fl.Config_kP(0, 0.06, 10);
  fl.Config_kI(0, 0.0, 10);
  fl.Config_kD(0, 0.6, 10);
  fl.ConfigMotionCruiseVelocity(1500, 10);
  fl.ConfigMotionAcceleration(700, 10);

  //Back Right Motion Magic Configs
  br.SelectProfileSlot(0, 0);
  br.Config_kF(0, 0.53, 10);
  br.Config_kP(0, 0.06, 10);
  br.Config_kI(0, 0.0, 10);
  br.Config_kD(0, 0.6, 10);
  br.ConfigMotionCruiseVelocity(1500, 10);
  br.ConfigMotionAcceleration(700, 10);

  //Back Left Motion Magic Configs
  bl.SelectProfileSlot(0, 0);
  bl.Config_kF(0, 0.54  , 10);
  bl.Config_kP(0, 0.06, 10);
  bl.Config_kI(0, 0.0, 10);
  bl.Config_kD(0, 0.6, 10);
  bl.ConfigMotionCruiseVelocity(1500, 10);
  bl.ConfigMotionAcceleration(700, 10);

  secondaryShooter.Follow(mainShooter, true);
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
  botpose_red = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose_wpired",std::vector<double>(6));
  botpose_blue = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose_wpiblue",std::vector<double>(6));
  bool llhastarget = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0.0);
  
  //frc::CameraServer::StartAutomaticCapture();

  //frc::SmartDashboard::PutNumber("Red", ColorSensor.GetRawColor().red);
  //frc::SmartDashboard::PutNumber("Green", ColorSensor.GetRawColor().green);
  //frc::SmartDashboard::PutNumber("Blue", ColorSensor.GetRawColor().blue);
  rotation = gyro.GetRotation2d();
  //double yaw = gyro.GetAngle();
  //units::degree_t rotationvalue = rotation.Degrees();
  //units::second_t time = frc::Timer::GetFPGATimestamp();
  //frc::SmartDashboard::PutNumber("rotation2d", rotationvalue.value());
  //units::angle::degree_t yawrotation(yaw);

  robotAngle = frc::InputModulus<units::degree_t>(
    rotation.Degrees(), halfangle2, halfangle);

    //robotAngle = units::angle::degree_t(robotAngle.value()-(time.value()*0.0425));

   m_poseEstimator.Update(
    frc::Rotation2d{robotAngle},
    frc::MecanumDriveWheelPositions{
      units::meter_t{(((fl.GetSelectedSensorPosition(0))/4096)*0.635)},
      units::meter_t{(((fr.GetSelectedSensorPosition(0))/4096)*-0.635)},
      units::meter_t{(((bl.GetSelectedSensorPosition(0))/4096)*0.635)},
      units::meter_t{(((br.GetSelectedSensorPosition(0))/4096)*-0.635)}
    }
  );
units::meter_t botX{botpose_blue[0]};
units::meter_t botY{botpose_blue[1]};
units::degree_t botRotation{botpose_blue[5]};
frc::SmartDashboard::PutNumber("tagnum", botpose_blue[7]);

frc::Pose2d visionMeasurement2d(botX,botY,frc::Rotation2d{robotAngle});
if(nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0) == 1){
  //gyro.SetYaw(botRotation);
  m_poseEstimator.AddVisionMeasurement(
    visionMeasurement2d,
    frc::Timer::GetFPGATimestamp()
  );
}
 

  frc::SmartDashboard::PutData("Field", &m_field);

  //auto camPoseEstimate = camPoseEstimator.Update();

  //wpi::array visionStdDevs{camPoseEstimate->estimatedPose.X().value()/2, camPoseEstimate->estimatedPose.Y().value()/2, 100.0};

  //if(camPoseEstimate){
   // m_poseEstimator.AddVisionMeasurement(camPoseEstimate->estimatedPose.ToPose2d(), camPoseEstimate->timestamp, visionStdDevs);
  //}

  
  //double camEstimatedX = camPoseEstimate->estimatedPose.X().value();
  //double camEstimatedY = camPoseEstimate->estimatedPose.Y().value();
  //double camEstimatedZ = camPoseEstimate->estimatedPose.Z().value();
  //double camEstimatedBearing = (camPoseEstimate->estimatedPose.Rotation().Z().value() / Pi * 180);


  //frc::SmartDashboard::PutNumber("camEstimatedX", camEstimatedX);
  //frc::SmartDashboard::PutNumber("camEstimatedY", camEstimatedY);
  //frc::SmartDashboard::PutNumber("camEstimatedZ",camEstimatedZ);
  //frc::SmartDashboard::PutNumber("camEstimatedBearing",camEstimatedBearing);

  
  m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());

  frc::SmartDashboard::PutNumber("Heading", gyro.GetAngle());

  m_field.GetRobotPose();

  //units::length::meter_t distance;
  //distance = frc::Pose2d::X;
  //distance.value();

  robotPose = m_field.GetRobotPose();

  robotX = robotPose.X();
  robotY = robotPose.Y();

  

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
  //m_autoSelected = frc::SmartDashboard::GetString("Auto Selector",
      //kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameMid) {
    // Custom Auto goes here
  }
  else if(m_autoSelected == kAutoNameR){

  }
  else {
    // Default Auto goes here
  }
  shooterDelay.Start();
  shooterDelay.Stop();
  shooterDelay.Reset();


  autoStep = 1;

  targetXPose = 49.833;
}

void Robot::AutonomousPeriodic() {

if(frc::DriverStation::GetAlliance() == frc::DriverStation::kRed){
if(m_autoSelected == kAutoNameMid){

if(autoStep == 1){
	ar.SetControl(m_request.WithPosition(39_tr));
	mainShooter.Set(.6);
	shooterDelay.Start();
	autoStep = 2;
}
else if(autoStep == 2 && shooterDelay.HasElapsed(2_s)){
	intakeMain.Set(.6);
	intakeFollow.Set(.6);
	autoStep = 3;
}
else if(autoStep == 3 && shooterDelay.HasElapsed(2.5_s)){
	intakeMain.Set(0);
	intakeFollow.Set(0);
  mainShooter.Set(0);
	ar.SetControl(m_request.WithPosition(1_tr));
	shooterDelay.Stop();
	shooterDelay.Reset();
	autoStep = 4;
}
else if(autoStep == 4){
	targetXPose = 42.5;

	robotXError = targetXPose - (robotX.value()*3.28);

	if(robotXError > 1){

		autoSpeedMulti = 1;

	}
	else if(robotXError < -1) {

		autoSpeedMulti = -1;

	}
	else{
		autoSpeedMulti = robotXError;
	}

	intakeMain.Set(.6);
	intakeFollow.Set(.6);

	fl.Set(ControlMode::PercentOutput, autoSpeedMulti*-.23);
	fr.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);
	bl.Set(ControlMode::PercentOutput, autoSpeedMulti*-.23);
	br.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);

	if(!stopSensor.Get() || abs(robotXError) < .25){
		autoStep = 5;
    fl.Set(ControlMode::PercentOutput, 0);
    fr.Set(ControlMode::PercentOutput, 0); 
    bl.Set(ControlMode::PercentOutput, 0);
    br.Set(ControlMode::PercentOutput, 0);
	}
}
else if(autoStep == 5){
	targetXPose = 49.883;

	robotXError = targetXPose - (robotX.value()*3.28);

	if(robotXError > 1){

		autoSpeedMulti = 1;

	}
	else if(robotXError < -1) {

		autoSpeedMulti = -1;

	}
	else{
		autoSpeedMulti = robotXError;
	}

	intakeMain.Set(.0);
	intakeFollow.Set(.0);

	fl.Set(ControlMode::PercentOutput, autoSpeedMulti*-.23);
	fr.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);
	bl.Set(ControlMode::PercentOutput, autoSpeedMulti*-.23);
	br.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);

	if(abs(robotXError) < .25){
		autoStep = 6;
    fl.Set(ControlMode::PercentOutput, 0);
    fr.Set(ControlMode::PercentOutput, 0); 
    bl.Set(ControlMode::PercentOutput, 0);
    br.Set(ControlMode::PercentOutput, 0);
	}
}

}
else if (autoStep == 6){
	ar.SetControl(m_request.WithPosition(39_tr));
	mainShooter.Set(.6);
	shooterDelay.Start();
	autoStep = 7;
}
else if(autoStep == 7 && shooterDelay.HasElapsed(2_s)){
	intakeMain.Set(.6);
	intakeFollow.Set(.6);
	autoStep = 8;
}
else if(autoStep == 8 && shooterDelay.HasElapsed(2.5_s)){
	intakeMain.Set(0);
	intakeFollow.Set(0);
  mainShooter.Set(0);
	ar.SetControl(m_request.WithPosition(1_tr));
	shooterDelay.Stop();
	shooterDelay.Reset();
	autoStep = 9;
}

}
else if (m_autoSelected == kAutoNameR){

if(autoStep == 1){
	ar.SetControl(m_request.WithPosition(39_tr));
	mainShooter.Set(.6);
	shooterDelay.Start();
	autoStep = 2;
}
else if(autoStep == 2 && shooterDelay.HasElapsed(2_s)){
	intakeMain.Set(.6);
	intakeFollow.Set(.6);
	autoStep = 3;
}
else if(autoStep == 3 && shooterDelay.HasElapsed(2.5_s)){
	intakeMain.Set(0);
	intakeFollow.Set(0);
  mainShooter.Set(0);
	ar.SetControl(m_request.WithPosition(1_tr));
	shooterDelay.Stop();
	shooterDelay.Reset();
	autoStep = 4;
}
else if(autoStep == 4){

	targetXPose = sqrt(3) * ((13.5 - 4.5)* 3.28);
	robotXError = targetXPose - (robotX.value()*3.28);
	

	if(robotXError > 1){

		autoSpeedMulti = 1;

	}
	else if(robotXError < -1) {

		autoSpeedMulti = -1;

	}
	else{
		autoSpeedMulti = robotXError;
	}

	fl.Set(ControlMode::PercentOutput, autoSpeedMulti*-.23);
	fr.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);
	bl.Set(ControlMode::PercentOutput, autoSpeedMulti*-.23);
	br.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);

	if(abs(robotXError) < .25){
		
		finalTurnPose = (robotX.value()*3.28);

		autoStep = 5;
    
    fl.Set(ControlMode::PercentOutput, 0);
    fr.Set(ControlMode::PercentOutput, 0); 
    bl.Set(ControlMode::PercentOutput, 0);
    br.Set(ControlMode::PercentOutput, 0);
	}
}
else if (autoStep == 5){
	
	targetRotation = 0;

	robotRotError = (targetRotation - robotAngle.value()) * .5;

	if(robotRotError > 1){

		autoSpeedMulti = 1;

	}
	else if(robotRotError < -1) {

		autoSpeedMulti = -1;

	}
	else{
		autoSpeedMulti = robotRotError;
	}
	

	fl.Set(ControlMode::PercentOutput, autoSpeedMulti*.23);
	fr.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);
	bl.Set(ControlMode::PercentOutput, autoSpeedMulti*.23);
	br.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);

	if(abs(robotRotError) < 2.5){
		autoStep = 6;
    fl.Set(ControlMode::PercentOutput, 0);
    fr.Set(ControlMode::PercentOutput, 0); 
    bl.Set(ControlMode::PercentOutput, 0);
    br.Set(ControlMode::PercentOutput, 0);
	}
}
else if(autoStep == 6){

	targetXPose = 42.5;

	robotXError = targetXPose - (robotX.value()*3.28);

	if(robotXError > 1){

		autoSpeedMulti = 1;

	}
	else if(robotXError < -1) {

		autoSpeedMulti = -1;

	}
	else{
		autoSpeedMulti = robotXError;
	}

	intakeMain.Set(.6);
	intakeFollow.Set(.6);

	fl.Set(ControlMode::PercentOutput, autoSpeedMulti*-.23);
	fr.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);
	bl.Set(ControlMode::PercentOutput, autoSpeedMulti*-.23);
	br.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);
	
	if(!stopSensor.Get() || abs(robotXError) < .25){
		autoStep = 7;
    fl.Set(ControlMode::PercentOutput, 0);
    fr.Set(ControlMode::PercentOutput, 0); 
    bl.Set(ControlMode::PercentOutput, 0);
    br.Set(ControlMode::PercentOutput, 0);
	}
}
else if (autoStep == 7){

	targetXPose = finalTurnPose;

	robotXError = targetXPose - (robotX.value()*3.28);

	if(robotXError > 1){

		autoSpeedMulti = 1;

	}
	else if(robotXError < -1) {

		autoSpeedMulti = -1;

	}
	else{
		autoSpeedMulti = robotXError;
	}

	intakeMain.Set(0);
	intakeFollow.Set(0);
  mainShooter.Set(0);

	fl.Set(ControlMode::PercentOutput, autoSpeedMulti*-.23);
	fr.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);
	bl.Set(ControlMode::PercentOutput, autoSpeedMulti*-.23);
	br.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);

	if(abs(robotXError) < .25){
		autoStep = 8;
    fl.Set(ControlMode::PercentOutput, 0);
    fr.Set(ControlMode::PercentOutput, 0); 
    bl.Set(ControlMode::PercentOutput, 0);
    br.Set(ControlMode::PercentOutput, 0);
    
	}
}
else if (autoStep == 8){
	//Could be -60
	targetRotation = 60;

	robotRotError = targetRotation - robotAngle.value();

	if(robotRotError > 1){

		autoSpeedMulti = 1;

	}
	else if(robotRotError < -1) {

		autoSpeedMulti = -1;

	}
	else{
		autoSpeedMulti = robotRotError;
	}

	fl.Set(ControlMode::PercentOutput, autoSpeedMulti*.23);
	fr.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);
	bl.Set(ControlMode::PercentOutput, autoSpeedMulti*.23);
	br.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);

	if(abs(robotRotError) < 2.5){
		autoStep = 9;
  fl.Set(ControlMode::PercentOutput, 0);
    fr.Set(ControlMode::PercentOutput, 0); 
    bl.Set(ControlMode::PercentOutput, 0);
    br.Set(ControlMode::PercentOutput, 0);
	}
}
else if (autoStep == 9){
	targetXPose = 16 * 3.28;

	robotXError = targetXPose - robotX.value();

	if(robotXError > 1){

		autoSpeedMulti = 1;

	}
	else if(robotXError < -1) {

		autoSpeedMulti = -1;

	}
	else{
		autoSpeedMulti = robotXError;
	}

	fl.Set(ControlMode::PercentOutput, autoSpeedMulti*-.23);
	fr.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);
	bl.Set(ControlMode::PercentOutput, autoSpeedMulti*-.23);
	br.Set(ControlMode::PercentOutput, autoSpeedMulti* .3);

	if(abs(robotXError) < .25){
		autoStep = 10;
	}
}
else if(autoStep == 10){
	ar.SetControl(m_request.WithPosition(39_tr));
	mainShooter.Set(.6);
	shooterDelay.Start();
	autoStep = 10;
}
else if(autoStep == 11 && shooterDelay.HasElapsed(2_s)){
	intakeMain.Set(.6);
	intakeFollow.Set(.6);
	autoStep = 12;
}
else if(autoStep == 12 && shooterDelay.HasElapsed(2.5_s)){
	intakeMain.Set(0);
	intakeFollow.Set(0);
  mainShooter.Set(0);
	ar.SetControl(m_request.WithPosition(1_tr));
	shooterDelay.Stop();
	shooterDelay.Reset();
	autoStep = 13;
}
}
else if (m_autoSelected == kAutoNameS){
if(autoStep == 1){
	ar.SetControl(m_request.WithPosition(39_tr));
	mainShooter.Set(.6);
	shooterDelay.Start();
	autoStep = 2;
}
else if(autoStep == 2 && shooterDelay.HasElapsed(2_s)){
	intakeMain.Set(.6);
	intakeFollow.Set(.6);
	autoStep = 3;
}
else if(autoStep == 3 && shooterDelay.HasElapsed(2.5_s)){
	intakeMain.Set(0);
	intakeFollow.Set(0);
  mainShooter.Set(0);
	ar.SetControl(m_request.WithPosition(1_tr));
	shooterDelay.Stop();
	shooterDelay.Reset();
	autoStep = 4;
}
}
}


void Robot::TeleopInit() {
    /*m_odometry.ResetPosition(
    gyro.GetRotation2d(),
    frc::MecanumDriveWheelPositions{
      units::meter_t{(((fl.GetSelectedSensorPosition(0))/4096)*25)},
      units::meter_t{(((fr.GetSelectedSensorPosition(0))/4096)*25)},
      units::meter_t{(((bl.GetSelectedSensorPosition(0))/4096)*25)},
      units::meter_t{(((br.GetSelectedSensorPosition(0))/4096)*25)}
    },
    frc::Pose2d(0_m, 0_m, 0_rad)
  );*/
  //gyro.Reset();
  //gyro.SetYaw(-60_deg);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",1);

  climbTimer.Start();
  climbTimer.Stop();
  climbTimer.Reset();
}

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

  xbox.GetPOV();

  //Target Direction, Magnitude, and Turn
  direction = atan2(ly,lx);
  magnitude = hypot(lx,ly);
  turn = rx*0.7;

  double pGyroYaw = pGyro.GetYaw();

  frc::SmartDashboard::PutNumber("pGyroYaw", pGyroYaw);

  frc::SmartDashboard::PutBoolean("ISstop", stopSensor.Get());

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



  //drive code
  if(!XButton){
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
  }
  frc::Pose2d robotPose = m_field.GetRobotPose();

  units::length::meter_t robotX = robotPose.X();
  units::length::meter_t robotY = robotPose.Y();

  //camtoTarget = botpose_red[0];
  //camtoTarget = camtoTarget - 0.2;
  if(frc::DriverStation::GetAlliance() == frc::DriverStation::kRed){
    camtoTarget = 16.5608-robotX.value();
  }else if(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue){
    camtoTarget = robotX.value()-0.381;
  }
  double targetdist;

  //target X 16.5608
  //target Y 5.53720l

  //if(nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tid", 0.00) == 4.0){

    targetdist = sqrt(pow(camtoTarget, 2)+pow(robotY.value()-5.5372, 2));
  //}else{
    //targetdist = sqrt(pow((robotX.value()-5.5372), 2)+pow(robotY.value(), 2));
  //};

  bool hastarget = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0);

  //shootangle = ((0.00002004*(pow(camtoTarget, 3)))+(-0.006432*(pow(camtoTarget, 2)))+(0.8477*camtoTarget)+25.12);
  
  //shootangle = ((1.223*(pow(camtoTarget, 3)))+(-9.969*(pow(camtoTarget, 2)))+(33.37*camtoTarget)+25.12);
  shootangle = ((1.223*(pow(targetdist, 3)))+(-9.969*(pow(targetdist, 2)))+(33.37*targetdist)+25.12);
  
  if(shootangle > 90){
    shootangle = 90;  
  }else if(shootangle < 39){
    shootangle = 39;
  }

  if(targetdist > 5){
    shootangle = 39;
  }

  double robotshootangle = (robotAngle.value() - atan(robotY.value()-5.5372/(robotX.value())));
  
  frc::SmartDashboard::PutNumber("camtotarget", camtoTarget);

  //units::angle::turn_t offset{(1.1111111111111111111*(pGyroYaw - shootangle))};
  units::angle::turn_t offset{(1.11111*shootangle)-15};
  frc::SmartDashboard::PutNumber("shootangle", shootangle);
  frc::SmartDashboard::PutNumber("shootangleoffset", (1.11111*shootangle)+1);
  //frc::SmartDashboard::PutNumber("offset", (1.1111111111111111111*(pGyroYaw - shootangle)));

  //arm controls
  if(RightBumper){

  //ar.SetControl(m_request.WithPosition(39_tr));

  ar.SetControl(m_request.WithPosition(offset));

  }
  if(RightTrigger){
    offset+=1_tr;
    ar.SetControl(m_request.WithPosition(offset));
  }
  if(LeftTrigger){
    offset-=1_tr;
    ar.SetControl(m_request.WithPosition(offset));
  }

  if(BackButton){

    ar.SetControl(m_request.WithPosition(25_tr));

  }

  if(LeftBumper){

  ar.SetControl(m_request.WithPosition(1_tr));

  }

  if(XButton){
  
  ar.SetControl(m_request.WithPosition(130_tr));

  if(frc::DriverStation::GetAlliance() == frc::DriverStation::kRed){
    ampError =  48.23 - robotX.value()*3.28;
  }
  else if(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue){
    ampError =  6.04 - robotX.value()*3.28;
  }
  else{
    ampError = 0;
  }

  if(ampError > 1){
    ampSpeedMulti = 1;
  }
  else if (ampError < -1){
    ampSpeedMulti = -1;
  }
  else{
    ampSpeedMulti = ampError;
  }

  fl.Set(ControlMode::PercentOutput, ampSpeedMulti*-.23);
	fr.Set(ControlMode::PercentOutput, ampSpeedMulti* .3);
	bl.Set(ControlMode::PercentOutput, ampSpeedMulti*.23);
	br.Set(ControlMode::PercentOutput, ampSpeedMulti* -.3);
  
  }
  //if(Dpad() == "Up"){
    //gyro.Reset();
  //}

  //Intake Statements
  if(YButton){
    pickupTimer.Start();
    intakeMain.Set(.5);
    intakeFollow.Set(.3);
    pickupActive = 1;
    xbox.SetRumble(frc::GenericHID::kBothRumble, 1);
  }
  if(!stopSensor.Get()){
    intakeMain.Set(0);
    intakeFollow.Set(0);
    pickupTimer.Stop();
    pickupTimer.Reset();
    pickupActive = 0;
    xbox.SetRumble(frc::GenericHID::kBothRumble, 0);
  }
  
  
  if(pickupTimer.HasElapsed(2.3_s)){
    intakeMain.Set(0);
    intakeFollow.Set(0);
    pickupActive = 0;
    xbox.SetRumble(frc::GenericHID::kBothRumble, 0);
  }
  else if(pickupTimer.HasElapsed(2_s)){
    intakeMain.Set(-.25);
    intakeFollow.Set(-.25);
  }


  //If intaking, dont shoot; Shooter statements
  if(!pickupActive){
    if(StartButton){
      mainShooter.Set(.75);
     shooterDelay.Start();
    }

    if(shooterDelay.HasElapsed(2_s)){
      mainShooter.Set(0);
      intakeMain.Set(0);
      intakeFollow.Set(0);
      shooterDelay.Stop();
      shooterDelay.Reset();
    }
    else if(shooterDelay.HasElapsed(1_s)){
      intakeMain.Set(.75);
      intakeFollow.Set(.75);
    }
  }

  //If intaking, dont shoot; Shooter statements
  if(!pickupActive){
    if(RightStickButton){
      mainShooter.Set(.75);
      shooterDelay.Start();
    }

    if(shooterDelay.HasElapsed(2_s)){
      mainShooter.Set(0);
      intakeMain.Set(0);
      intakeFollow.Set(0);
      shooterDelay.Stop();
      shooterDelay.Reset();
    }
    else if(shooterDelay.HasElapsed(1_s)){
      intakeMain.Set(.65);
      intakeFollow.Set(.65);
    }
  }

  //Push Note Back out of Intake
  if(!pickupActive){
    if(AButton){
      pickupTimer.Start();
      intakeMain.Set(-.7);
      intakeFollow.Set(-.7);
    }
    if(pickupTimer.HasElapsed(.05_s)){
      intakeMain.Set(0);
      intakeFollow.Set(0);
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

  if(Dpad() == "Right"){

    activeDriverCam = !activeDriverCam;

    if(activeDriverCam){
      //if driverCam State 1 switch to cam 1
      cameraSelection.SetString(camera1.GetName());
      //DriverFeed.SetSource(camera1);
    }
    else if(!activeDriverCam){
      //if driverCam State 0 switch to cam 0
      cameraSelection.SetString(camera2.GetName());
      //DriverFeed.SetSource(camera2);
    }
  }
  if(Dpad() == "Left"){
    ar.SetControl(m_request.WithPosition(101_tr));
  }

  if(Dpad() == "Down"){
    cl.Set(ControlMode::PercentOutput, .6);
    cr.Set(ControlMode::PercentOutput, -.6);
  }
  else if(Dpad() == "Up"){
    cl.Set(ControlMode::PercentOutput, -.3);
    cr.Set(ControlMode::PercentOutput, .3);
  }
  else{
    cl.Set(ControlMode::PercentOutput, 0);
    cr.Set(ControlMode::PercentOutput, 0);
  }

  //limelight angle is 61 degrees
  //smol: 1.5 inch radius, 9.42 in circumference
  //beeg: 4 inch radius, 25.13 in circumference
  //4:1 gear ratio
  //100:1 gearbox
  //400:1 full gear ratio
  //current angle: pGyroYaw;
  //1 turn = 0.9 degrees

}


std::string Robot::Dpad(){
  //Check Up Down Left and Right
  switch(xbox.GetPOV()){
    case 0:
      return "Up"; break;
    case 180:
      return "Down"; break;
    case 270:
      return "Left"; break;
    case 90:
      return "Right"; break;
    case 45:
      return "UpRight"; break;
    case 135:
      return "DownRight"; break;
    case 225:
      return "DownLeft"; break;
    case 315:
      return "UpLeft"; break;
    default:
      return "None";
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
