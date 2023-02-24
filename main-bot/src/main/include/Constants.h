// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace AutoConstants
{
   constexpr double kAutoBalanceMargin = 0.5;
   constexpr double kAutoBalnace_kP = 0.2;
   constexpr double kAutoBalance_kD = 0.01; // total guess
   constexpr double kAutoBalance_MaxOutput = 0.4;

   constexpr double kAutoRotateToTag_kP = 0.03;
   constexpr double kAutoRotateMaxSpeed = 0.5;
   constexpr double kAutoRotateDegreeMargin = 3.0;
};

namespace OperatorConstants
{

   constexpr int kJoyLPort = 0;
   constexpr int kJoyRPort = 1;
   constexpr int kXboxPort = 2;
   constexpr double kXboxStickDeadzone = 0.07;

} // namespace OperatorConstants

namespace DriveTrainConstants
{
   constexpr int kPigin_CANID = 20;
   constexpr int pdp_CANID = 25;
   constexpr int kDriveNEOL1_CANID = 4;
   constexpr int kDriveNEOL2_CANID = 5;
   constexpr int kDriveNEOL3_CANID = 6;
   constexpr int kDriveNEOR1_CANID = 1;
   constexpr int kDriveNEOR2_CANID = 2;
   constexpr int kDriveNEOR3_CANID = 3;
}

namespace ArmConstants
{
   constexpr int kArmSolenoid1 = 2;
   constexpr int kArmSolenoid2 = 3;
   constexpr int kClawSolenoid1 = 0;
   constexpr int kClawSolenoid2 = 1;
   constexpr int kWrist_CANID = 7;
   constexpr int kElevator_CANID = 8;

   // constexpr int elevator_DIOPIN = 0;
   constexpr int kWrist_DIOPIN = 0;
   constexpr int kElevatorLimitDown_DIOPIN = 1;
   constexpr int kElevatorLimitUp_DIOPIN = 2;

   constexpr double kElevatorPowerLimit = 0.75;

   constexpr double kWristSpeedScaleFactor = 0.5;
   
   constexpr double kElevatorAllowedError = 50.0;
   constexpr double kElevatorEncDistPerRev = 5.62345; // Inches per rev of 15T sprocket (1.79 PD)
   constexpr double kElevator_P = 0.2;
   constexpr double kElevator_I = 0.0;
   constexpr double kElevator_D = 2.0;
   constexpr double kElevator_F = 0.0;
   constexpr int kTimeoutMs = 30;
   constexpr int kElevatorPIDID = 0;

   constexpr int kElevatorPos_HIGH = 4000;
   constexpr int kElevatorPos_MID = 2000;
   constexpr int kElevatorPos_LOW = 1000;
   constexpr int kElevatorPos_DOWN = 0;

}

namespace AutoConstants
{
   constexpr double auto_balance_margin = 0.5;
   constexpr double autoBalnace_kP = 0.2;
   constexpr double autoBalance_kD = 0.01; // total guess
   constexpr double autoBalance_MaxOutput = 0.4;
};
