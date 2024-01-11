// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateToAngle.h"

RotateToAngle::RotateToAngle(DriveTrain *m_drivetrain, double m_angle, double m_speed, double m_timeout)
    : m_drivetrain(m_drivetrain), m_angle(m_angle), m_speed(m_speed), m_timeout(m_timeout)
{
       SetName("RotateToAngle");
    AddRequirements({m_drivetrain});
}
// Called when the command is initially scheduled.
void RotateToAngle::Initialize() 
{
    m_drivetrain->GyroSetTargetAngle(m_angle);
    m_timer.Reset();
    m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void RotateToAngle::Execute() 
{
   m_drivetrain->GyroTurnToTargetAngle();
}

// Called once the command ends or is interrupted.
void RotateToAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool RotateToAngle::IsFinished() {
   if (m_timer.HasElapsed(units::time::second_t(m_timeout)))
    {
        return m_drivetrain->GyroIsAtHeading(kGyroRotateAngleBandWide);
    }
    else
    {
        return m_drivetrain->GyroIsAtHeading(kGyroRotateAngleBand);
    }
}
