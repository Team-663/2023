// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"

DriveTrain::DriveTrain()
{
   m_driveSpeed = 0.0;
   m_pigeon.SetYawToCompass();
}

void DriveTrain::SetMotorSpeed(double speed)
{
   m_driveSpeed = speed;
}

void DriveTrain::SetMotorAngle(double angle)
{
   m_driveAngle = angle;
}

void DriveTrain::SwerveDrive(units::meters_per_second_t x, units::meters_per_second_t y, units::radians_per_second_t z, bool fieldRelative)
{
      auto moduleStates = m_kinematics.ToSwerveModuleStates(
          fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(x, y, z,frc::Rotation2d(units::degree_t{m_pigeon.GetYaw()}))
                        : frc::ChassisSpeeds{x, y, z});
                     

   m_kinematics.DesaturateWheelSpeeds(&moduleStates, DriveTrainConstants::kDriveModuleMaxSpeed);

   frc::SmartDashboard::PutNumber("xSpeed", x.value());
   frc::SmartDashboard::PutNumber("ySpeed", y.value());
   frc::SmartDashboard::PutNumber("zRotation", z.value());

   auto [frontRight, rearRight, frontLeft, rearLeft] = moduleStates;

   m_frontRight.SetDesiredState(frontRight);
   m_rearRight.SetDesiredState(rearRight);
   m_frontLeft.SetDesiredState(frontLeft);
   m_rearLeft.SetDesiredState(rearLeft);

}

// This method will be called once per scheduler run
void DriveTrain::Periodic()
{
#ifdef TEST_MODE
   m_swerve1Drive.SetVoltage(m_driveSpeed * 12.0_V);
#endif
}
