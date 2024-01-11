// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmByJoystick.h"

ArmByJoystick::ArmByJoystick(Arm *m_arm, frc2::CommandXboxController* m_xbox,frc::Joystick* m_joyL, frc::Joystick* m_joyR)//double m_leftY, double m_rightY, int m_pov)
    : m_arm(m_arm)
    ,m_xbox(m_xbox)
    ,m_joyL(m_joyL)
    ,m_joyR(m_joyR)
    //,m_leftY(m_leftY),m_rightY(m_rightY),m_pov(m_pov)
{
   AddRequirements({m_arm});
   // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ArmByJoystick::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ArmByJoystick::Execute()
{
   /*
   //RobotContainer *robot = RobotContainer::GetInstance();

   double leftY = robot->GetXbox()->GetLeftY();
   double rightY = robot->GetXbox()->GetRightY();
   bool bumpL = robot->GetXbox()->GetLeftBumper();
   bool bumpR = robot->GetXbox()->GetRightBumper();
   int dpad = robot->GetXbox()->GetPOV();
   */
   //double leftY = m_xbox->GetLeftY();
   double rightY = m_xbox->GetRightY();
   double rightYDead = 0.0;
   int dpad = m_xbox->GetPOV();
   bool bumpL = m_xbox->GetLeftBumper();
   bool bumpR = m_xbox->GetRightBumper();
   //double triggerL = m_xbox->GetLeftTriggerAxis();
   double triggerR = m_xbox->GetRightTriggerAxis();
   //double leftJoystickY = m_xbox->GetLeftY();
   //double leftJoystickX = m_xbox->GetLeftX();
   bool a = m_xbox->GetAButton();
   bool b = m_xbox->GetBButton();
   bool x = m_xbox->GetXButton();
   bool y = m_xbox->GetYButton();
   bool start = m_xbox->GetStartButton();
   //frc::SmartDashboard::PutNumber("Arm Lefty:", leftY);
   //frc::SmartDashboard::PutNumber("Arm RightY:", rightY);

   if (rightY > kXboxStickDeadzone || rightY < -kXboxStickDeadzone)
      rightYDead = rightY;
   
   if (rightYDead > 0.0)
   {
      m_arm->ElevatorSetPIDState(false);
   }
   m_arm->SetElevatorSpeedManual(-rightYDead);
/*
   if (leftJoystickY > kLeftJoystickSensitivity)
   {
      m_arm->Extend();
   }
   else if (leftJoystickY < -kLeftJoystickSensitivity)
   {
      m_arm->Retract();
   }
   */
   if (m_joyL->GetTrigger() || m_joyR->GetTrigger())
   {
      m_arm->Extend();
   }

   if (m_joyL->GetRawButton(2) || m_joyR->GetRawButton(2))
   {
      m_arm->Retract();
   }

   if (triggerR > kTriggerSensitivity)
   {
         m_arm->OpenClaw();
   }
   else
   {
      m_arm->CloseClaw();
   }

   if (a ^ b ^ x ^ y)
   {
      if (a)
      {
         m_arm->UpdateElevatorSetpoint(kElevatorPos_LOW);
         m_arm->ElevatorSetPIDState(true);
      }

      if (b)
      {
         m_arm->UpdateElevatorSetpoint(kElevatorPos_MID);
         m_arm->ElevatorSetPIDState(true);
      }

      if (y)
      {
          m_arm->UpdateElevatorSetpoint(kElevatorPos_HIGH);
         m_arm->ElevatorSetPIDState(true);
         
      }
   }

   switch (dpad)
   {
      case 0:
         m_arm->UpdateWristSetpoint(kWristSetpointBack);
         break;
      case 90:
         m_arm->UpdateWristSetpoint(kWristSetpointStraight);
         break;
      case 180:
         m_arm->UpdateWristSetpoint(kWristSetpointDown);
         break;
      case 270:
         m_arm->UpdateWristSetpoint(kWristSetpointPickup);
         break;
   }

   if (bumpL ^ bumpR)
   {
      if (bumpL)
         m_arm->SetWristSpeed(-kWristUpSpeed);
      if (bumpR)
         //m_arm->SetWristSpeed(-kWristUpSpeed);
         m_arm->SetWristSpeed(kWristDownSpeed);
   }
   else
   {
      m_arm->SetWristSpeed(0);
   }


   if(start)
      m_arm->ResetElevatorEncoder();

   if(m_xbox->GetBackButton())
      m_arm->ResetWristEncoder();

}

// Called once the command ends or is interrupted.
void ArmByJoystick::End(bool interrupted) {}

// Returns true when the command should end.
bool ArmByJoystick::IsFinished()
{
   return false;
}
