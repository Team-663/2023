// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>


#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer* RobotContainer::m_robotContainer = NULL;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  frc::SmartDashboard::PutNumber("TEST NUM", 123);
  frc::SmartDashboard::PutData("Auto Balance", new AutoBalance(&m_driveTrain));
  frc::SmartDashboard::PutData("Auto Rotate To Tag", new RotateToTag(&m_driveTrain, false));
  //frc::SmartDashboard::Put("Open Claw", frc2::cmd::Run([this] { m_arm.OpenClaw(); }, {&m_arm}));

  

  // Configure the button bindings
  ConfigureBindings();

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
   //m_arm.ElevatorManualSpeed(m_xbox.GetRightY()));
}

void RobotContainer::ConfigureBindings()
{

   // frc2::JoystickButton(&m_xbox,
  //                    frc::XboxController::Button::kA)
   //   .WhileTrue(RotateToTag(&m_driveTrain, true).ToPtr());
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  //m_xbox.A().WhileActiveOnce(m_)
  m_xbox.A().WhileTrue(RotateToTag(&m_driveTrain, true).ToPtr());
  m_xbox.LeftBumper().OnTrue(frc2::cmd::Run([this] { m_arm.OpenClaw(); }, {&m_arm}));
  m_xbox.RightBumper().OnTrue(frc2::cmd::Run([this] { m_arm.CloseClaw(); }, {&m_arm}));
  m_xbox.B().OnTrue(m_arm.ResetElevatorEncoderCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}

RobotContainer* RobotContainer::GetInstance()
{
    if (m_robotContainer == NULL) {
        m_robotContainer = new RobotContainer();
    }
    return(m_robotContainer);
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
