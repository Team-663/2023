// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveStraightDistance.h"

DriveStraightDistance::DriveStraightDistance(DriveTrain* m_drivetrain)
:m_drivetrain(m_drivetrain)
{
   AddRequirements({m_drivetrain});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void DriveStraightDistance::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveStraightDistance::Execute()
{

}

// Called once the command ends or is interrupted.
void DriveStraightDistance::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveStraightDistance::IsFinished() {
  return false;
}
