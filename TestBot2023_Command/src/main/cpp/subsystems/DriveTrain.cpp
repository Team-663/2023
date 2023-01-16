// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"
#include <frc/smartdashboard/SmartDashboard.h>

DriveTrain::DriveTrain() 
: m_pigeon{20}
{
    
}
// This method will be called once per scheduler run
void DriveTrain::Periodic()
{
    frc::SmartDashboard::PutNumber("PIG Gyro:", m_pigeon.GetYaw());
}

void DriveTrain::TankDrive(double l, double r)
{
    frc::SmartDashboard::PutNumber("Tank L", l);
    frc::SmartDashboard::PutNumber("Tank R", r);
}
