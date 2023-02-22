// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Wrist.h"

Wrist::Wrist()
{
   m_wrist.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      m_wrist.SetInverted(false); // TODO: update

}

void Wrist::MoveWrist(double val)
{
   m_wristSpeed = val;
   m_wrist.Set(m_wristSpeed);
}

// This method will be called once per scheduler run
void Wrist::Periodic()
{
   frc::SmartDashboard::PutNumber("Wrist Speed", m_wristSpeed);
}
