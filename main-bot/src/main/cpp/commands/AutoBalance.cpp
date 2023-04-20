// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoBalance.h"

AutoBalance::AutoBalance(DriveTrain* m_drivetrain):
m_drivetrain(m_drivetrain)
{
   AddRequirements({m_drivetrain});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoBalance::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoBalance::Execute() {

      double pitch = 0.0;
#ifdef USE_XBOX_TRIGGER_AS_PITCH
   RobotContainer* robot = RobotContainer::GetInstance();
   pitch = robot->GetXbox()->GetLeftTriggerAxis() + robot->GetXbox()->GetRightTriggerAxis();
   frc::SmartDashboard::PutNumber("Dummy Pitch Value", pitch);
#else
   pitch = m_drivetrain->GyroGetPitch();
#endif

   m_error = -pitch;

   double output = m_error * kAutoBalnace_kP * kAutoBalance_MaxOutput;
   frc::SmartDashboard::PutNumber("AutoBalance Output", output);
}

// Called once the command ends or is interrupted.
void AutoBalance::End(bool interrupted) {
   m_drivetrain->Stop();
}

// Returns true when the command should end.
bool AutoBalance::IsFinished() {
  return (abs(m_error) < kAutoBalanceMargin);;
}
