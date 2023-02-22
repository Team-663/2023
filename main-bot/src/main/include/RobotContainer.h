// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/Joystick.h>
#include <subsystems/Arm.h>
#include <subsystems/DriveTrain.h>
#include <subsystems/Camera.h>
#include <subsystems/Wrist.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include "Constants.h"

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
   frc2::CommandXboxController* GetXbox();
   frc::Joystick* GetJoyL();
   frc::Joystick* GetJoyR();
   frc2::CommandPtr GetAutonomousCommand();

 private:
   frc::Joystick m_joyL{kJoyLPort};
   frc::Joystick m_joyR{kJoyRPort};
   frc2::CommandXboxController m_xbox{kXboxPort};

   // The robot's subsystems are defined here...
   DriveTrain m_driveTrain;
   Arm m_arm;
   Wrist m_wrist;

  void ConfigureBindings();
  double Deadzone(double input);
};
