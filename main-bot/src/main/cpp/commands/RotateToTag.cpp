// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateToTag.h"

using namespace AutoConstants;

RotateToTag::RotateToTag(DriveTrain *m_drivetrain, Camera* m_camera, bool m_continuous) : m_drivetrain(m_drivetrain)
{
   AddRequirements({m_drivetrain});
   AddRequirements({m_camera});
   // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RotateToTag::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RotateToTag::Execute()
{
   m_error = m_camera->GetTargetXOffSet();
   double HasTarget = m_camera->GetCameraValidTargets();
   frc::SmartDashboard::PutNumber("autoRotateErr", m_error);
   if (HasTarget == 1.0)
   {
      double diffSpeed = fabs(m_error) * kAutoRotateToTag_kP;
      double mult = 1.0;
      if (m_error < 0.0)
      {
         mult = -1.0;
      }

      if (diffSpeed > kAutoRotateMaxSpeed)
      {
         diffSpeed = kAutoRotateMaxSpeed;
      }

      m_drivetrain->TankDrive(mult * diffSpeed, -mult * diffSpeed);
      frc::SmartDashboard::PutNumber("diffSpeed", diffSpeed);
   }
   else
   {
      m_drivetrain->Stop();
   }
}

// Called once the command ends or is interrupted.
void RotateToTag::End(bool interrupted) {}

// Returns true when the command should end.
bool RotateToTag::IsFinished()
{
   if (m_continuous)
   {
      return false;
   }
   else
   {
      if (fabs(m_error) < kAutoRotateDegreeMargin)
      {
         return true;
      }
      {
         return false;
      }
   }
}
