// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/Encoder.h>
#include "Constants.h"
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
//#include "wpi/span.h"

class DriveTrain : public frc2::SubsystemBase {
   public:
   DriveTrain();

   /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
   void Periodic() override;
   void TankDrive(double l, double r);
   void Stop();
   double GetPitch();
   double GetTargetXOffSet();
   double GetCameraValidTargets();

 private:
   // Components (e.g. motor controllers and sensors) should generally be
   // declared private and exposed only through public methods.
   rev::CANSparkMax m_driveMotorL1;
   rev::CANSparkMax m_driveMotorL2;
   //rev::CANSparkMax m_driveMotorL3;

   rev::CANSparkMax m_driveMotorR1;
   rev::CANSparkMax m_driveMotorR2;
   //rev::CANSparkMax m_driveMotorR3;

   Pigeon2 m_pigeon;

   // The motors on the left side of the drive
   frc::MotorControllerGroup m_leftMotors{m_driveMotorL1, m_driveMotorL2};

   // The motors on the right side of the drive
   frc::MotorControllerGroup m_rightMotors{m_driveMotorR1, m_driveMotorR2};

   // The robot's drive
   frc::DifferentialDrive m_drive{m_leftMotors, m_rightMotors};
  
};
