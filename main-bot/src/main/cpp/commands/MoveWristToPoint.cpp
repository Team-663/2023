// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveWristToPoint.h"

MoveWristToPoint::MoveWristToPoint(Arm * m_arm, double m_setpoint, double m_timeout)
: m_arm(m_arm), m_setpoint(m_setpoint), m_timeout(m_timeout)
{
   AddRequirements({m_arm});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void MoveWristToPoint::Initialize() 
{
   m_arm->UpdateWristSetpoint(m_setpoint);
}

// Called repeatedly when this Command is scheduled to run
void MoveWristToPoint::Execute() 
{

}

// Called once the command ends or is interrupted.
void MoveWristToPoint::End(bool interrupted) {}

// Returns true when the command should end.
bool MoveWristToPoint::IsFinished() {
  return m_arm->IsWristAtSetpoint();
}
