// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"

using namespace DriveTrainConstants;

DriveTrain::DriveTrain():
   m_driveMotorL1{kDriveNEOL1_CANID, rev::CANSparkMax::MotorType::kBrushless},
   m_driveMotorL2{kDriveNEOL2_CANID, rev::CANSparkMax::MotorType::kBrushless},
   m_driveMotorL3{kDriveNEOL3_CANID, rev::CANSparkMax::MotorType::kBrushless},
   m_driveMotorR1{kDriveNEOR1_CANID, rev::CANSparkMax::MotorType::kBrushless},
   m_driveMotorR2{kDriveNEOR2_CANID, rev::CANSparkMax::MotorType::kBrushless},
   m_driveMotorR3{kDriveNEOR3_CANID, rev::CANSparkMax::MotorType::kBrushless},

   m_pigeon{kPigin_CANID}
{

}

// This method will be called once per scheduler run
void DriveTrain::TankDrive(double l, double r)
{
   
   m_driveLVal = l;
   m_driveRVal = r;

   m_drive.TankDrive(l, r, true);
}

void DriveTrain::Stop()
{
   TankDrive(0,0);
}

double DriveTrain::GetPitch()
{
   return m_pigeon.GetPitch();
}

void DriveTrain::Periodic() 
{
   DisplayValues();
}

void DriveTrain::DisplayValues()
{
   frc::SmartDashboard::PutNumber("Gyro Yaw:", m_pigeon.GetYaw());
   frc::SmartDashboard::PutNumber("Gyro Pitch:", m_pigeon.GetPitch());
     frc::SmartDashboard::PutNumber("Tank L", m_driveLVal);
   frc::SmartDashboard::PutNumber("Tank R", m_driveRVal);
   /*
   std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
   double targets = table->GetNumber("tv",0.0);
   double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
   double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
   frc::SmartDashboard::PutNumber("Cam Has Target", targets);
   frc::SmartDashboard::PutNumber("Cam X", targetOffsetAngle_Horizontal);
   frc::SmartDashboard::PutNumber("Cam Y", targetOffsetAngle_Vertical);
   */
}
