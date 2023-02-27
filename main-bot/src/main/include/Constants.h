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
#define WRIST_USE_DEG_CONVERSION 0
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
   constexpr double kXboxStickDeadzone = 0.06;

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

   constexpr double kGyroRotateKP = 0.035;          // was 0.3
   constexpr double kGyroRotateAngleBand = 3.5;     // Rotate to within 3 degrees
   constexpr double kGyroRotateAngleBandWide = 6.0; // if all else fails, be somewhere close...
   constexpr double kGyroMaxTurnRate = 0.45;        // set to .55 for carpet

   constexpr double kGyroDriveKP = 0.01;
   constexpr double kAutoStraightSpeed = 0.6;

   constexpr double kDriveStraightKP = 0.01; // proportional gain coefficient for auto driving

   constexpr double kDriveRampRateTeleop = 0.0;
   constexpr double kDriveRampRateAuto = 0.5;

   constexpr double kDriveAutoErrorMargin = 100.0; // TODO: tune
   constexpr double kDriveAutoMaxOutput = 0.5;
   constexpr double kDriveAutoProportionalDist = 1000.0; // only start proportional descent within this many error units

}

namespace ArmConstants
{
   constexpr int kArmSolenoid1 = 4;
   constexpr int kArmSolenoid2 = 6;
   constexpr int kClawSolenoid1 = 5;
   constexpr int kClawSolenoid2 = 7;
   constexpr int kWrist_CANID = 7;
   constexpr int kElevator_CANID = 8;

   // constexpr int elevator_DIOPIN = 0;
   constexpr int kWrist_DIOPIN = 0;
   constexpr int kElevatorLimitDown_DIOPIN = 1;
   constexpr int kElevatorLimitUp_DIOPIN = 2;

   constexpr double kWrist_P = 0.1;
   constexpr double kWrist_I = 0.0;
   constexpr double kWrist_D = 1.0;
   constexpr double kWrist_F = 0.0;
   
#if (WRIST_USE_DEG_CONVERSION == 1)
   constexpr double kWristPosConversion = 1.0; // turn 0-1 into degrees
#else
   constexpr double kWristPosConversion = 360.0; // turn 0-1 into degrees
#endif

// Setpoints for wrist
   constexpr double kWristSetpointDown = 180 / kWristPosConversion;
   constexpr double kWristSetpointStraight = 150 / kWristPosConversion;
   constexpr double kWristSetpointUp =  85 / kWristPosConversion;
   constexpr double kWristSetpointBack = 55 / kWristPosConversion;
   constexpr double kWristSoftLimReverse = 60  / kWristPosConversion;
   constexpr double kWristSoftLimForward = 180 /  kWristPosConversion;
   constexpr double kWristInFrameAngle = 90  / kWristPosConversion;

   
   constexpr double kWristAllowedError = 10 / kWristPosConversion;
   constexpr double kWristJoyRotateScaler = 10.0;  // NEED TO TUNE: rate of turn per joystick scale
   constexpr double kWristSpeedScaleFactor = 0.5;
   constexpr double kWristUpSpeed = 0.40;
   constexpr double kWristDownSpeed = 0.20;

   constexpr double kTriggerSensitivity = 0.3; //Nathan added this
   constexpr double kLeftJoystickSensitivity = 0.85;

   constexpr double kElevatorPowerLimit = 0.95;
   constexpr double kElevatorPowerLimitDown = 0.35;
   constexpr bool   kElevatorIsMotorInverted = false;
   constexpr double kElevatorAllowedError = 250.0;
   constexpr double kElevatorEncDistPerRev = 5.62345; // Inches per rev of 15T sprocket (1.79 PD)
   constexpr double kElevatorTicsPerInch = 727.214;
   constexpr double kElevator_P = 0.2;
   constexpr double kElevator_I = 0.00015;
   constexpr double kElevator_D = 2.0;
   constexpr double kElevator_F = 0.0;
   constexpr int kTimeoutMs = 30;
   constexpr int kElevatorPIDID = 0;

   constexpr int kElevatorPos_HIGH = 24500.0;
   constexpr int kElevatorPos_MID = 10000.0;
   constexpr int kElevatorPos_LOW = 100;
   constexpr int kElevatorPos_DOWN = 0;

}

namespace AutoConstants
{
   constexpr double auto_balance_margin = 0.5;
   constexpr double autoBalnace_kP = 0.2;
   constexpr double autoBalance_kD = 0.01; // total guess
   constexpr double autoBalance_MaxOutput = 0.4;
};
