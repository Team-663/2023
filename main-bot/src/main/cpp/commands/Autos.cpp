// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"

frc2::CommandPtr autos::ExampleAuto(ExampleSubsystem* subsystem) {
  return frc2::cmd::Sequence(subsystem->ExampleMethodCommand(),
                             ExampleCommand(subsystem).ToPtr());
}

frc2::CommandPtr autos::DriveDistanceCmd(DriveTrain * drivetrain, double dist)
{
   return frc2::cmd::Sequence(
      drivetrain->DriveStraightCmd(dist, 0.5)
      
   );
}


frc2::CommandPtr autos::AutoScoreOnMidCmd(Arm* arm, DriveTrain * drivetrain)
{
   return frc2::cmd::Sequence(
      arm->SetElevatorPositionCmd(ArmConstants::kElevatorPos_HIGH)
      ,arm->ExtendCmd()
      ,frc2::cmd::Wait(0.5_s)
      ,arm->SetWristPositionCmd(ArmConstants::kWristSetpointDown)
      ,arm->OpenClawCmd()
      ,frc2::cmd::Wait(0.5_s)      
      ,arm->SetWristPositionCmd(ArmConstants::kWristSetpointBack)
      ,frc2::cmd::Wait(0.5_s) 
      ,arm->CloseClawCmd()
      ,frc2::cmd::Wait(0.5_s) 
      ,arm->RetractCmd()
      ,frc2::cmd::Wait(0.5_s)
      ,arm->SetElevatorPositionCmd(ArmConstants::kElevatorPos_LOW)
      );
}

frc2::CommandPtr autos::AutoScoreAndBackAwayCmd(Arm *arm, DriveTrain* drivetrain, double dist)
{
   return frc2::cmd::Sequence(
      AutoScoreOnMidCmd(arm, drivetrain)
      ,DriveDistanceCmd(drivetrain, dist)
      
   );
}
