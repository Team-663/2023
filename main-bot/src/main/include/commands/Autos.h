// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/ExampleSubsystem.h"
#include <subsystems/Arm.h>
#include <subsystems/DriveTrain.h>


namespace autos {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr DriveDistanceCmd(DriveTrain * drivetrain, double dist);
frc2::CommandPtr AutoScoreOnMidCmd(Arm* arm, DriveTrain *drivetrain);
frc2::CommandPtr AutoScoreAndBackAwayCmd(Arm *arm, DriveTrain* drivetrain, double dist);

frc2::CommandPtr ExampleAuto(ExampleSubsystem* subsystem);
}  // namespace autos
