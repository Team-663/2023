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

#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <cmath>

/**
 * This header contains hold robot-wide numerical or boolean constants ONLY.
 * 
 * Place constants into subsystem/command -specific NAMESPACES within this
 * header, which can then be included (where they are needed).
 */

namespace controllerConstants {
    //USB port addresses on drivestation PC.
    constexpr int kControllerMainID = 0;
    constexpr int kControllerAuxID = 1;
}

namespace drivetrainConstants {
    //CAN IDs
    constexpr int kMotorDriveFrontRightID = 0;
    constexpr int kMotorDriveRearRightID = 1;
    constexpr int kMotorDriveFrontLeftID = 2;
    constexpr int kMotorDriveRearLeftID = 3;

    constexpr int kMotorTurnFrontRightID = 4;
    constexpr int kMotorTurnRearRightID = 5;
    constexpr int kMotorTurnFrontLeftID = 6;
    constexpr int kMotorTurnRearLeftID = 7;

    constexpr int kEncoderTurnFrontRightID = 8;
    constexpr int kEncoderTurnRearRightID = 9;
    constexpr int kEncoderTurnFrontLeftID = 10;
    constexpr int kEncoderTurnRearLeftID = 11;

    namespace offsets {
        constexpr double kFrontRight{54.844};
        constexpr double kRearRight{-154.951};
        constexpr double kFrontLeft{-139.922};
        constexpr double kRearLeft{103.008};
    }

    namespace swerveModules {
        constexpr double kModuleFrontRight[4]{kMotorDriveFrontRightID,
                                                   kMotorTurnFrontRightID,
                                                   kEncoderTurnFrontRightID,
                                                   offsets::kFrontRight};
        constexpr double kModuleRearRight[4]{kMotorDriveRearRightID,
                                                  kMotorTurnRearRightID,
                                                  kEncoderTurnRearRightID,
                                                  offsets::kRearRight};
        constexpr double kModuleFrontLeft[4]{kMotorDriveFrontLeftID,
                                                  kMotorTurnFrontLeftID,
                                                  kEncoderTurnFrontLeftID,
                                                  offsets::kFrontLeft};
        constexpr double kModuleRearLeft[4]{kMotorDriveRearLeftID,
                                                 kMotorTurnRearLeftID,
                                                 kEncoderTurnRearLeftID,
                                                 offsets::kRearLeft};
    }

    namespace calculations {
        constexpr auto kFinalDriveRatio{6.75 * 360_deg};
        constexpr units::length::inch_t kWheelCircumference = {2 * M_PI * 3.8_in / 2};

        constexpr auto kModuleMaxSpeed{16.3_fps};
        constexpr auto kChassisMaxSpeed{16.3_fps};

        constexpr auto kModuleMaxAngularVelocity{M_PI * 1_rad_per_s};  // radians per second
        constexpr auto kModuleMaxAngularAcceleration{M_PI * 2_rad_per_s / 1_s};  // radians per second^2

        constexpr double kMotorMaxOutput = 0.5;
        constexpr double kMotorDeadband = 0.1;
    }
}
