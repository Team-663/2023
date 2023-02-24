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

using namespace DriveTrainConstants;
using namespace AutoConstants;

class DriveTrain : public frc2::SubsystemBase {
public:
   DriveTrain();

   void Periodic() override;
   void TankDrive(double l, double r);
   void Stop();
   double GetPitch();
   double GetTargetXOffSet();
   double GetCameraValidTargets();
   

 private:
   rev::CANSparkMax m_driveMotorL1;
   rev::CANSparkMax m_driveMotorL2;
   rev::CANSparkMax m_driveMotorL3;

   rev::CANSparkMax m_driveMotorR1;
   rev::CANSparkMax m_driveMotorR2;
   rev::CANSparkMax m_driveMotorR3;

   Pigeon2 m_pigeon;

   // The motors on the left side of the drive
   frc::MotorControllerGroup m_leftMotors{m_driveMotorL1, m_driveMotorL2, m_driveMotorL3};

   // The motors on the right side of the drive
   frc::MotorControllerGroup m_rightMotors{m_driveMotorR1, m_driveMotorR2, m_driveMotorR3};

   // The robot's drive
   frc::DifferentialDrive m_drive{m_leftMotors, m_rightMotors};

   double m_driveLVal;
   double m_driveRVal;

   void DisplayValues();
};
