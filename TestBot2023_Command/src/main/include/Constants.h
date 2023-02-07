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

//#define USE_XBOX_TRIGGER_AS_PITCH 1

namespace OperatorConstants {

constexpr int kJoyLPort = 0;
constexpr int kJoyRPort = 1;
constexpr int kXboxPort = 2;

}  // namespace OperatorConstants

namespace DriveTrainConstants
{
    constexpr int pigin_CANID = 20;
    constexpr int pdp_CANID = 25;
    constexpr int driveNEOL1_CANID = 1;
    constexpr int driveNEOL2_CANID = 2;
    constexpr int driveNEOR1_CANID = 3;
    constexpr int driveNEOR2_CANID = 4;
}

namespace ArmConstants
{
    constexpr int compressor_CANID = 0;
    constexpr int extendoArm_solenoid1 = 2;
    constexpr int extendoArm_solenoid2 = 3;
    constexpr int claw_solenoid1 = 0;
    constexpr int claw_solenoid2 = 1;
    constexpr int wristbow_CANID = 7;
    constexpr int elevator_CANID = 8;

    constexpr int elevator_DIOPIN = 0;
    
    constexpr double autoRotateToTag_kP = 0.03;
    constexpr double autoRotateMaxSpeed = 0.5;
    constexpr double autoRotateDegreeMargin = 3.0;

}

namespace AutoConstants
{
    constexpr double auto_balance_margin = 0.5;
    constexpr double autoBalnace_kP = 0.2;
    constexpr double autoBalance_kD = 0.01; // total guess
    constexpr double autoBalance_MaxOutput = 0.4;
};
