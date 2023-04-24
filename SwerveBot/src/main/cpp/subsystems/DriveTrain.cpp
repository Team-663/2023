// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"

DriveTrain::DriveTrain()
{
   m_driveSpeed = 0.0;
}

void DriveTrain::SetMotorSpeed(double speed)
{
   m_driveSpeed = speed;
}

void DriveTrain::SetMotorAngle(double angle)
{
   m_driveAngle = angle;
}

void DriveTrain::SwerveDrive(units::feet_per_second x, units::feet_per_second y, units::degrees_per_second)
{

}

// This method will be called once per scheduler run
void DriveTrain::Periodic()
{
#ifdef TEST_MODE
   m_swerve1Drive.SetVoltage(m_driveSpeed * 12.0_V);
#endif
}
