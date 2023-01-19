// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoBalance.h"

using namespace AutoConstants;

AutoBalance::AutoBalance(DriveTrain* m_driveTrain) 
   : m_driveTrain(m_driveTrain)
{

    SetName("DriveByJoystick");
    AddRequirements({m_driveTrain});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoBalance::Initialize() 
{
   m_targetPitch = 0.0;
}

// Called repeatedly when this Command is scheduled to run
void AutoBalance::Execute()
{


   double pitch = 0.0;
#ifdef USE_XBOX_TRIGGER_AS_PITCH
   RobotContainer* robot = RobotContainer::GetInstance();
   pitch = robot->GetXbox()->GetLeftTriggerAxis() + robot->GetXbox()->GetRightTriggerAxis();
   frc::SmartDashboard::PutNumber("Dummy Pitch Value", pitch);
#else
   pitch = m_driveTrain->GetPitch();
#endif

   m_error = m_targetPitch - pitch;

   double output = m_error * autoBalnace_kP * autoBalance_MaxOutput;
   frc::SmartDashboard::PutNumber("AutoBalance Output", output);


}

// Called once the command ends or is interrupted.
void AutoBalance::End(bool interrupted) 
{
   m_driveTrain->Stop();
}

// Returns true when the command should end.
bool AutoBalance::IsFinished() {
   return (abs(m_error) < 0.5);
}
