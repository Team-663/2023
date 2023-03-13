// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer *RobotContainer::m_robotContainer = NULL;

RobotContainer::RobotContainer()
{

   frc::CameraServer::StartAutomaticCapture();
   m_chooser.SetDefaultOption("Score Cone and Backup", m_scoreBackAuto.get());
   m_chooser.AddOption("Score Cone", m_scoreAuto.get());
   m_chooser.AddOption("Score And Balance", m_ScoreCubeAndBalanceAuto.get());
   m_chooser.AddOption("Balance", m_BalanceAuto.get());
   m_chooser.AddOption("Score Cube", m_ScoreCubeAuto.get());
   m_chooser.AddOption("Rotate 180", m_rotate180Auto.get());
   
   frc::Shuffleboard::GetTab("Autonomous").Add(m_chooser).WithSize(3, 1);

   frc::SmartDashboard::PutData("Rotate 180", m_rotate180Auto.get());
   frc::Shuffleboard::GetTab("Autonomous").Add("Balance", m_BalanceAuto.get());
   m_drivetrain.SetDefaultCommand(DriveByJoystick(&m_drivetrain, &m_joyL, &m_joyR));
   m_arm.SetDefaultCommand(ArmByJoystick(&m_arm, &m_xbox, &m_joyL, &m_joyR));
   // Configure the button bindings
   ConfigureBindings();
}

RobotContainer *RobotContainer::GetInstance()
{
   if (m_robotContainer == NULL)
   {
      m_robotContainer = new RobotContainer();
   }
   return (m_robotContainer);
}

void RobotContainer::ConfigureBindings()
{

   // m_xbox.X().OnTrue(m_drivetrain.BalanceOnRampCmd(0.4));
   // TODO: enable this when ready
   // m_xbox.X().WhileTrue(RotateToTag(&m_drivetrain, &m_camera, true).ToPtr());

   /* DO NOT USE: these interrupt the ArmByJoystick command */
   /*
     m_xbox.LeftBumper().OnTrue(frc2::cmd::Run([this] { m_arm.OpenClaw(); }, {&m_arm}));
     m_xbox.RightBumper().OnTrue(frc2::cmd::Run([this] { m_arm.CloseClaw(); }, {&m_arm}));
     m_xbox.Y().OnTrue(frc2::cmd::Run([this] { m_arm.Extend(); }, {&m_arm}));
     m_xbox.A().OnTrue(frc2::cmd::Run([this] { m_arm.Retract(); }, {&m_arm}));

     m_xbox.Back().OnTrue(m_arm.ResetElevatorEncoderCommand());
     */
}

void RobotContainer::SetMotorsToTeleopSettings()
{
   m_drivetrain.SetDrivetrainRamprate(kDriveRampRateTeleop);
   m_drivetrain.SetMotorMode(false);
}

void RobotContainer::SetWristDefaults()
{
   m_arm.SetWristSetpointHere();
}

// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
frc2::Command *RobotContainer::GetAutonomousCommand()
{
   // An example command will be run in autonomous
   // return autos::ExampleAuto(&m_subsystem);
   return m_chooser.GetSelected();
}

frc2::CommandXboxController *RobotContainer::GetXbox()
{
   return &m_xbox;
}
frc::Joystick *RobotContainer::GetJoyR()
{
   return &m_joyR;
}
frc::Joystick *RobotContainer::GetJoyL()
{
   return &m_joyL;
}

double RobotContainer::Deadzone(double input)
{
   if (input < kXboxStickDeadzone && input > -kXboxStickDeadzone)
      return 0.0;
   return input;
}
