// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"


RobotContainer::RobotContainer() {
  frc::SmartDashboard::PutNumber("TEST NUM", 123);
  //frc::SmartDashboard::PutData("Auto Balance", new AutoBalance(&m_driveTrain));
  //frc::SmartDashboard::PutData("Auto Rotate To Tag", new RotateToTag(&m_driveTrain, false));
  
  m_driveTrain.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        //m_driveTrain.TankDrive(-m_xbox.GetLeftY(),
        //                    -m_xbox.GetRightY());
        m_driveTrain.TankDrive(m_joyL.GetY(), m_joyR.GetY());
      },
      {&m_driveTrain}));

  m_arm.SetDefaultCommand(frc2::cmd::Run(
      [this] {
         m_arm.SetElevatorSpeedManual(m_xbox.GetRightY()); 
      }, 
      {&m_arm}));

  m_wrist.SetDefaultCommand(frc2::cmd::Run(
      [this] {
         m_wrist.MoveWrist(m_xbox.GetLeftY()); 
      }, 
      {&m_wrist}));
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  //m_xbox.A().WhileTrue(RotateToTag(&m_driveTrain, true).ToPtr());
  m_xbox.LeftBumper().OnTrue(frc2::cmd::Run([this] { m_arm.OpenClaw(); }, {&m_arm}));
  m_xbox.RightBumper().OnTrue(frc2::cmd::Run([this] { m_arm.CloseClaw(); }, {&m_arm}));
  m_xbox.B().OnTrue(m_arm.ResetElevatorEncoderCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  //return autos::ExampleAuto(&m_subsystem);
}

double RobotContainer::Deadzone(double input)
{
  if (input < kXboxStickDeadzone && input > -kXboxStickDeadzone)
    return 0.0;
  return input;
}
