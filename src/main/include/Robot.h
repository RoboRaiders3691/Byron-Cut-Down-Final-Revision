// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <vector>
#include <utility>
#include <frc/MathUtil.h>

#include <frc/TimedRobot.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/Odometry.h>
#include <frc/kinematics/Kinematics.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveOdometry.h>
#include <frc/estimator/MecanumDrivePoseEstimator.h>

#include <frc/XboxController.h>
#include <frc/GenericHID.h>

#include "ctre/Phoenix.h"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/Pigeon2.hpp"
#include "ctre/phoenix6/configs/Configurator.hpp"
#include "ctre/phoenix6/configs/Configs.hpp"

#include "rev/CANSparkMax.h"
#include "rev/CANSparkLowLevel.h"
#include "rev/ColorSensorV3.h"

#include <units/angle.h>
#include <units/length.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include "frc/Timer.h"
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>



#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
//#include "LimelightHelpers.h"
#include <cameraserver/CameraServer.h>

#include "frc/DigitalInput.h"
#include "wpi/SpanExtras.h"
#include <wpi/SymbolExports.h>



#define WPILIB_DLLEXPORT
class WPILIB_DLLEXPORT ObjectToRobotPose;

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  std::string Dpad();
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  //Setting Up Camera Streams
  cs::UsbCamera camera1;
  cs::UsbCamera camera2;

  //cs::MjpegServer DriverFeed = frc::CameraServer::AddSwitchedCamera("DriverCamFeed");

  nt::NetworkTableEntry cameraSelection;

  bool activeDriverCam = false;

  //Main Drivetrain
  TalonSRX fl{6};
  TalonSRX fr{5};
  TalonSRX bl{1};
  TalonSRX br{0};

  ctre::phoenix6::controls::MotionMagicVoltage m_request{0_tr};

  //Arm
  ctre::phoenix6::hardware::TalonFX al{7};
  ctre::phoenix6::hardware::TalonFX ar{8};

  //Shoot/Intake
  rev::CANSparkMax secondaryShooter{10, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax mainShooter{13, rev::CANSparkLowLevel::MotorType::kBrushless};
  //rev::CANSparkMax intakeMain{12, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax intakeFollow{12, rev::CANSparkLowLevel::MotorType::kBrushless};

  ctre::phoenix6::hardware::TalonFX intakeMain{15};

  //Color Sensor V3
  static constexpr auto i2cPort = frc::I2C::kOnboard;

  rev::ColorSensorV3 ColorSensor{i2cPort};

  frc::DigitalInput stopSensor{0};

  //Drive Multipliers
  double spdmult = 1;
  double topspeed = 1;

  //Shooter Variables
  double shootangle;
  double camtoTarget;

  //Stick Vars
  double lx = 0.0;
  double ly = 0.0;
  double rx = 0.0;
  double ry = 0.0;
  bool LeftStickButton = 0;
  bool RightStickButton = 0;
  
  //A, B, X, Y, Back, and Start

  bool AButton = 0;
  bool BButton = 0;
  bool XButton = 0;
  bool YButton = 0;
  bool StartButton = 0;
  bool BackButton = 0;

  //Trigers and Bumpers

  bool LeftBumper = 0;
  double LeftTrigger = 0;
  bool RightBumper = 0;
  double RightTrigger = 0;
  
 //Misc Vars
  double slider = 0.0;
  double spd = 0.0;
  double direction = 0.0;
  double magnitude = 0.0;
  double turn = 0.0;

  std::vector<double> botpose;
  //Variable for Pi
  const double Pi = 3.1415926535;

  //Instance of the Xbox Controller Class
  frc::XboxController xbox{0};

  //Instance of Pigeon2 Class
  phoenix6::hardware::Pigeon2 gyro{24};

  //PigeonIMU pGyro{4};

  //Instance of Field2d and Rotation Objects
  frc::Field2d m_field;

  units::degree_t halfangle{180};
  units::degree_t halfangle2{-180};
  units::degree_t robotAngle{0};

  frc::Rotation2d rotation = gyro.GetRotation2d();

  /*frc::Rotation2d halfangle{units::degree_t(180)};
  frc::Rotation2d halfangle2{units::degree_t(-180)};
  frc::Rotation2d robotAngle{};*/

  //Set up wheel locations
  frc::Translation2d m_frontLeftLocation{0.53416_m, 0.53416_m};
  frc::Translation2d m_frontRightLocation{0.53416_m, -0.53416_m};
  frc::Translation2d m_backLeftLocation{-0.53416_m, 0.53416_m};
  frc::Translation2d m_backRightLocation{-0.53416_m, -0.53416_m};

  //Create kinematics object using the wheel locations
  frc::MecanumDriveKinematics m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};

  frc::MecanumDrivePoseEstimator m_poseEstimator{
    m_kinematics,
    robotAngle,
    frc::MecanumDriveWheelPositions{
      units::meter_t{(((fl.GetSelectedSensorPosition(0))/4096)*0.635)},
      units::meter_t{(((fr.GetSelectedSensorPosition(0))/4096)*-0.635)},
      units::meter_t{(((bl.GetSelectedSensorPosition(0))/4096)*0.635)},
      units::meter_t{(((br.GetSelectedSensorPosition(0))/4096)*-0.635)}
    },
    frc::Pose2d{0_m, 0_m, 0_rad}
  };

  //Instance of PhotonCamera

 // photon::PhotonCamera pCamera1{"Microsoft_LifeCam_HD-3000"};


  //offset currently undecided
  frc::Transform3d robotToCam1 =
  frc::Transform3d(frc::Translation3d(0_m, 0_m, 0_m),
                    frc::Rotation3d(0_rad, 0_rad, 0_rad));
                    
   
  frc::Timer pickupTimer;
  frc::Timer shooterDelay;
  frc::Timer autoTimer;
  bool pickupActive= 0;

  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  int flipDrive = 1;

  double targetAngle = 0;


    frc::AprilTagFieldLayout aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

    //photon::PhotonPoseEstimator camPoseEstimator{
      //aprilTagFieldLayout, photon::MULTI_TAG_PNP_ON_COPROCESSOR, std::move(pCamera1), robotToCam1};

  double targetXPose = 0;
  double xPoseBlue = 0;
  double xRobotError = 0;
  double autoSpeedMulti = 0;

};