// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer* RobotContainer::m_robotContainer = NULL;

RobotContainer::RobotContainer() {
  frc::SmartDashboard::PutNumber("TEST NUM", 123);
  m_chooser.SetDefaultOption("AUTO: Drive Forward", new DriveStraightDistance(&m_drivetrain));
  //m_chooser.AddOption("AUTO: Drive Backwards", new DriveStraightDistance(&m_driveTrain, &m_shooter, -4, 0.6, false, false, 0.0));
  //m_chooser.AddOption("AUTO: Shoot and Back",  new ShootAndBackup(&m_shooter, &m_driveTrain));
  //m_chooser.SetDefaultOption("AUTO: 2 Ball", &m_autoTwoBall);
  
  frc::SmartDashboard::PutData("Auto Balance", new AutoBalance(&m_drivetrain));
  //frc::SmartDashboard::PutData("Auto Rotate To Tag", new RotateToTag(&m_driveTrain, false));
  
  m_drivetrain.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        //m_driveTrain.TankDrive(-m_xbox.GetLeftY(),
        //                    -m_xbox.GetRightY());
        m_drivetrain.TankDrive(m_joyL.GetY(), m_joyR.GetY());
      },
      {&m_drivetrain}));
   m_arm.SetDefaultCommand(ArmByJoystick(&m_arm));
/*
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
*/
  // Configure the button bindings
  ConfigureBindings();
}

RobotContainer* RobotContainer::GetInstance()
{
    if (m_robotContainer == NULL) {
        m_robotContainer = new RobotContainer();
    }
    return(m_robotContainer);
}

void RobotContainer::ConfigureBindings() {
  //m_xbox.s
  m_xbox.A().WhileTrue(RotateToTag(&m_drivetrain, &m_camera, true).ToPtr());
  //m_xbox.LeftBumper().OnTrue(frc2::cmd::Run([this] { m_arm.OpenClaw(); }, {&m_arm}));
  //m_xbox.RightBumper().OnTrue(frc2::cmd::Run([this] { m_arm.CloseClaw(); }, {&m_arm}));
  //m_xbox.B().OnTrue(m_arm.ResetElevatorEncoderCommand());
}

//frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  //return autos::ExampleAuto(&m_subsystem);
  return m_chooser.GetSelected();
}

frc2::CommandXboxController* RobotContainer::GetXbox() {
   return &m_xbox;
}
frc::Joystick* RobotContainer::GetJoyR() {
   return &m_joyR;
}
frc::Joystick* RobotContainer::GetJoyL() {
   return &m_joyL;
}

double RobotContainer::Deadzone(double input)
{
  if (input < kXboxStickDeadzone && input > -kXboxStickDeadzone)
    return 0.0;
  return input;
}
