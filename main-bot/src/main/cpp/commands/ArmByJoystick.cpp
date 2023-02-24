// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmByJoystick.h"

ArmByJoystick::ArmByJoystick(Arm *m_arm)
    : m_arm(m_arm)
{
   AddRequirements({m_arm});
   // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ArmByJoystick::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ArmByJoystick::Execute()
{
   RobotContainer *robot = RobotContainer::GetInstance();

   double leftY = robot->GetXbox()->GetLeftY();
   double rightY = robot->GetXbox()->GetRightY();
   bool bumpL = robot->GetXbox()->GetLeftBumper();
   bool bumpR = robot->GetXbox()->GetRightBumper();
   int dpad = robot->GetXbox()->GetPOV();

   m_arm->SetWristSpeed(leftY * kWristSpeedScaleFactor);
   m_arm->SetElevatorSpeedManual(rightY * kElevatorPowerLimit);

   switch (dpad)
   {
      case robot->DPAD_UP:
         m_arm->Extend();
         break;
      case robot->DPAD_DOWN:
         m_arm->Retract();
         break;
   }

   if (bumpL ^ bumpR)
   {
      if (bumpL)
         m_arm->OpenClaw();
      if (bumpR)
         m_arm->CloseClaw();
   }
}

// Called once the command ends or is interrupted.
void ArmByJoystick::End(bool interrupted) {}

// Returns true when the command should end.
bool ArmByJoystick::IsFinished()
{
   return false;
}
