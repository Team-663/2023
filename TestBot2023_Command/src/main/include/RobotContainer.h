// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc/Joystick.h>
#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/Arm.h"

#include <commands/AutoBalance.h>
#include <commands/RotateToTag.h>
#include <frc/shuffleboard/Shuffleboard.h>

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
   frc2::CommandPtr GetAutonomousCommand();
   frc2::CommandXboxController* GetXbox();
   frc::Joystick* GetJoyL();
   frc::Joystick* GetJoyR();
   static RobotContainer* GetInstance();

private:
   
   static RobotContainer* m_robotContainer;
   // Replace with CommandPS4Controller or CommandJoystick if needed
   frc::Joystick m_joyL{OperatorConstants::kJoyLPort};
   frc::Joystick m_joyR{OperatorConstants::kJoyRPort};
   frc2::CommandXboxController m_xbox{OperatorConstants::kXboxPort};

   // The robot's subsystems are defined here...
   ExampleSubsystem m_subsystem;
   DriveTrain m_driveTrain;
   Arm m_arm;

   void ConfigureBindings();

};
