// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateToTag.h"

RotateToTag::RotateToTag(DriveTrain* m_driveTrain, bool m_continuous) 
     : m_driveTrain(m_driveTrain)
{

    AddRequirements({m_driveTrain});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RotateToTag::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RotateToTag::Execute()
  {
     m_error=m_driveTrain->GetTargetXOffSet();
    double HasTarget=m_driveTrain->GetCameraValidTargets();
   frc::SmartDashboard::PutNumber("autoRotateErr", m_error);
 if (HasTarget==1.0)
  {
    double diffSpeed = fabs(m_error) * autoRotateToTag_kP;
    double mult = 1.0;
    if (m_error < 0.0)
    {
        mult = -1.0;
    }

    if (diffSpeed > autoRotateMaxSpeed)
    {
      diffSpeed = autoRotateMaxSpeed;
    }

    m_driveTrain->TankDrive(mult*diffSpeed, -mult*diffSpeed);
    frc::SmartDashboard::PutNumber("diffSpeed", diffSpeed);
  }
  else
  {
      m_driveTrain->Stop();
  }
  }

// Called once the command ends or is interrupted.
void RotateToTag::End(bool interrupted)
 {
    m_driveTrain->Stop();
 }

// Returns true when the command should end.
bool RotateToTag::IsFinished()
{
   if (m_continuous)
   {
      return false;
   }
   else
   {
      if (fabs(m_error)<autoRotateDegreeMargin)
      {
         return true;
      }
      {
         return false;
      }
   }
}
