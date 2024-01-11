// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include "units/velocity.h"
#include "units/length.h"
#include "units/angular_velocity.h"
#include "units/math.h"

class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SetMotorSpeed(double speed);
  void SetMotorAngle(double angle);

  void SwerveDrive(units::feet_per_second x, units::feet_per_second y, units::degrees_per_second);

 private:
  rev::CANSparkMax m_swerve1Drive;
  rev::CANSparkMax m_swerve1Steer;
  

  double m_driveSpeed;
  double m_driveAngle;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
