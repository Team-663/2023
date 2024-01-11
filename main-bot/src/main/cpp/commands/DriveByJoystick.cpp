// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveByJoystick.h"

DriveByJoystick::DriveByJoystick(DriveTrain *m_drivetrain, frc::Joystick *m_joyL, frc::Joystick *m_joyR)
    : m_drivetrain(m_drivetrain), m_joyL(m_joyL), m_joyR(m_joyR)
{
   AddRequirements({m_drivetrain});

   // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void DriveByJoystick::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveByJoystick::Execute()
{
   double left = m_joyL->GetY();
   double right = m_joyR->GetY();
   double avg = (left+right) / 2.0;

   if (m_joyL->GetRawButtonPressed(4))
   {
      m_drivetrain->GyroSetTargetAngleHere();
   }
   
   if (m_joyR->GetRawButtonReleased(6))
   {
      m_drivetrain->ToggleMotorNeutralMode();
   }
   
   if (m_joyL->GetRawButton(4))
   {
      m_drivetrain->GyroDriveStraight(avg);
       frc::SmartDashboard::PutBoolean("Gyro Drive?", true);
      // Something
   }
   else
   {
      m_drivetrain->TankDrive(left, right);
       frc::SmartDashboard::PutBoolean("Gyro Drive?", false);
   }
   frc::SmartDashboard::PutNumber("Drive Avg", avg);
}

// Called once the command ends or is interrupted.
void DriveByJoystick::End(bool interrupted) 
{
   m_drivetrain->Stop();
}

// Returns true when the command should end.
bool DriveByJoystick::IsFinished()
{
   return false;
}
