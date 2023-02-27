// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/FunctionalCommand.h>
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

class DriveTrain : public frc2::SubsystemBase
{
public:
   DriveTrain();

   void Periodic() override;
   void TankDrive(double l, double r);
   void Stop();
   double GyroGetPitch();

   void GyroResetHeading();
   void GyroSetTargetAngle(double tgtAngle);
   void GyroTurnToTargetAngle();
   void GyroDriveStraight(double speed, double angle);
   bool GyroIsAtHeading(double band);

   double GetDriveEncoderValue();
   void UpdateDriveSetpoint(double dist);
   bool IsDriveAtSetpoint();
   void DriveToSetpoint();
   void SetDrivetrainRamprate(double rate);

   frc2::CommandPtr DriveStraightCmd(double dist, double timeout);

private:
   rev::CANSparkMax m_driveMotorL1{kDriveNEOL1_CANID, rev::CANSparkMax::MotorType::kBrushless};
   rev::CANSparkMax m_driveMotorL2{kDriveNEOL2_CANID, rev::CANSparkMax::MotorType::kBrushless};
   rev::CANSparkMax m_driveMotorL3{kDriveNEOL3_CANID, rev::CANSparkMax::MotorType::kBrushless};

   rev::CANSparkMax m_driveMotorR1{kDriveNEOR1_CANID, rev::CANSparkMax::MotorType::kBrushless};
   rev::CANSparkMax m_driveMotorR2{kDriveNEOR2_CANID, rev::CANSparkMax::MotorType::kBrushless};
   rev::CANSparkMax m_driveMotorR3{kDriveNEOR3_CANID, rev::CANSparkMax::MotorType::kBrushless};

   Pigeon2 m_pigeon{kPigin_CANID};

   // The motors on the left side of the drive
   frc::MotorControllerGroup m_leftMotors{m_driveMotorL1, m_driveMotorL2, m_driveMotorL3};

   // The motors on the right side of the drive
   frc::MotorControllerGroup m_rightMotors{m_driveMotorR1, m_driveMotorR2, m_driveMotorR3};

   // The robot's drive
   frc::DifferentialDrive m_drive{m_leftMotors, m_rightMotors};

   double m_driveLVal;
   double m_driveRVal;
   double m_tgtAngle;
   double m_gyroTurnError;
   double m_driveSetpoint;
   double m_driveError;
   double m_isDriveAtSetpoint;

   void DisplayValues();
};
