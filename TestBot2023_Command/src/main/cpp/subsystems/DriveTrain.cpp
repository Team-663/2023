// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace DriveTrainConstants;

DriveTrain::DriveTrain() 
   : 
   m_driveMotorL1{driveNEOL1_CANID, rev::CANSparkMax::MotorType::kBrushless},
   m_driveMotorL2{driveNEOL2_CANID, rev::CANSparkMax::MotorType::kBrushless},
   m_driveMotorR1{driveNEOR1_CANID, rev::CANSparkMax::MotorType::kBrushless},
   m_driveMotorR2{driveNEOR2_CANID, rev::CANSparkMax::MotorType::kBrushless},

   m_pigeon{20}
{
   m_rightMotors.SetInverted(true);
   std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}
// This method will be called once per scheduler run
void DriveTrain::Periodic()
{
   frc::SmartDashboard::PutNumber("Gyro Yaw:", m_pigeon.GetYaw());
   frc::SmartDashboard::PutNumber("Gyro Pitch:", m_pigeon.GetPitch());
   std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
   double targets = table->GetNumber("tv",0.0);
   double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
   double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
   frc::SmartDashboard::PutNumber("Cam Has Target", targets);
   frc::SmartDashboard::PutNumber("Cam X", targetOffsetAngle_Horizontal);
   frc::SmartDashboard::PutNumber("Cam Y", targetOffsetAngle_Vertical);
   
   //frc::SmartDashboard::PutNumber("Enc L"
   //   , m_driveMotorL1.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor).GetPosition());

   //frc::SmartDashboard::PutNumber("Enc R"
   //   , m_driveMotorR1.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor).GetPosition());
}

double DriveTrain::GetTargetXOffSet()
{
   std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
   return table->GetNumber("tx",0.0);


}

double DriveTrain::GetCameraValidTargets()
{
   std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
   return table->GetNumber("tv",0.0);
}

void DriveTrain::TankDrive(double l, double r)
{
   frc::SmartDashboard::PutNumber("Tank L", l);
   frc::SmartDashboard::PutNumber("Tank R", r);

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
