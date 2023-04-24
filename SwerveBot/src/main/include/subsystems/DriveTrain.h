// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include "units/velocity.h"
#include "units/length.h"
#include "units/angular_velocity.h"
#include "units/math.h"
#include "SwerveModule.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace DriveTrainConstants;
//#define TEST_MODE true
class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SetMotorSpeed(double speed);
  void SetMotorAngle(double angle);

  void SwerveDrive(units::meters_per_second_t x, units::meters_per_second_t y, units::radians_per_second_t z, bool fieldRelative);

private:
#ifdef TEST_MODE
  rev::CANSparkMax m_swerve1Drive{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_swerve1Steer{2, rev::CANSparkMax::MotorType::kBrushless};
  Pigeon2 m_pigeon{kPidginID};
#else
  frc::Translation2d m_locationFrontRight{+11.6875_in, -11.6875_in};
  frc::Translation2d m_locationRearRight{-11.6875_in, -11.6875_in};
  frc::Translation2d m_locationFrontLeft{+11.6875_in, +11.6875_in};
  frc::Translation2d m_locationRearLeft{-11.6875_in, +11.6875_in};

  SwerveModule m_frontRight{kModuleFrontRightID, kMotorDriveFrontRightID, kMotorTurnFrontRightID, kEncoderTurnFrontRightID, kFrontRightOffset};
  SwerveModule m_rearRight{kModuleRearRightID, kMotorDriveRearRightID, kMotorTurnRearRightID, kEncoderTurnRearRightID, kRearRightOffset};
  SwerveModule m_frontLeft{kModuleFrontLeftID, kMotorDriveFrontLeftID, kMotorTurnFrontLeftID, kEncoderTurnFrontLeftID, kFrontLeftOffset};
  SwerveModule m_rearLeft{kModuleRearLeftID, kMotorDriveRearLeftID, kMotorTurnRearLeftID, kEncoderTurnRearLeftID, kRearLeftOffset};

  Pigeon2 m_pigeon{kPidginID};

  frc::SwerveDriveKinematics<4> m_kinematics{m_locationFrontRight,
                                             m_locationRearRight,
                                             m_locationFrontLeft,
                                             m_locationRearLeft};

  //frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_navX.GetRotation2d()};
#endif
  double m_driveSpeed;
  double m_driveAngle;


  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
