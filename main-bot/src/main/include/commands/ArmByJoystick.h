// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "RobotContainer.h"
#include <subsystems/Arm.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc/Joystick.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ArmByJoystick
    : public frc2::CommandHelper<frc2::CommandBase, ArmByJoystick>
{
public:
   // ArmByJoystick(Arm *m_arm, double m_leftY, double m_rightY, int m_pov);
   ArmByJoystick(Arm *m_arm, frc2::CommandXboxController* m_xbox,frc::Joystick* m_joyL, frc::Joystick* m_joyR);
   void Initialize() override;

   void Execute() override;

   void End(bool interrupted) override;

   bool IsFinished() override;

private:
   Arm *m_arm;
   frc2::CommandXboxController *m_xbox;
   frc::Joystick* m_joyL;
   frc::Joystick* m_joyR;
   double m_leftY;
   double m_rightY;
   int m_pov;
};
