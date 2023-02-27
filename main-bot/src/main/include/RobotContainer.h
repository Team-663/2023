// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <subsystems/Arm.h>
#include <subsystems/DriveTrain.h>
#include <subsystems/Camera.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include "Constants.h"

#include <commands/ArmByJoystick.h>
#include <commands/DriveStraightDistance.h>
#include <commands/AutoBalance.h>
#include <commands/RotateToTag.h>
#include <commands/MoveElevatorToPoint.h>
#include <commands/MoveWristToPoint.h>

using namespace OperatorConstants;
using namespace ArmConstants;

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
public:
   RobotContainer();
   static RobotContainer* GetInstance();
   frc2::CommandXboxController* GetXbox();
   frc::Joystick* GetJoyL();
   frc::Joystick* GetJoyR();
   frc2::Command* GetAutonomousCommand();

   void SetMotorsToTeleopSettings();

   enum E_DPAD_POV {
      DPAD_UP = 0,
      DPAD_RIGHT = 90,
      DPAD_DOWN = 180,
      DPAD_LEFT = 270,
      DPAD_NEUTRAL = -1
   };

 private:
   frc::Joystick m_joyL{kJoyLPort};
   frc::Joystick m_joyR{kJoyRPort};
   frc2::CommandXboxController m_xbox{kXboxPort};
   //frc::XboxController m_xbox{kXboxPort};

   // The robot's subsystems are defined here...
   DriveTrain m_drivetrain;
   Arm m_arm;
   Camera m_camera;
   static RobotContainer* m_robotContainer;
   frc::SendableChooser<frc2::Command*> m_chooser;
  void ConfigureBindings();
  double Deadzone(double input);
};
