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
#include <SwerveModule.h>

/**
 * This header contains hold robot-wide numerical or boolean constants ONLY.
 *
 * Place constants into subsystem/command -specific NAMESPACES within this
 * header, which can then be included (where they are needed).
 */

namespace controllerConstants
{
   // USB port addresses on drivestation PC.
   constexpr int kControllerMainID = 0;
   constexpr int kControllerAuxID = 1;
}

namespace DriveTrainConstants
{
   // Initial controller configs
   constexpr double kDriveMotorRampRate = 2.0;
   constexpr double kDrivePID_FF = 0.002;
   constexpr double kDriveCurrentLimit = 80.0;
   constexpr double kDriveFinalRatio = 6.75; // WRONG: TODO: calculate
   constexpr units::length::inch_t kDriveWheelCircumference = {2 * M_PI * 3.8_in / 2};
   constexpr auto kDriveModuleMaxSpeed{16.3_fps}; // WRONG: TODO: calculate
   constexpr auto kDriveChassisMaxSpeed{16.3_fps}; // WRONG: TODO: calculate


   // Start with these, from SDS/2190 code
   // See here: https://github.com/SwerveDriveSpecialties/swerve-lib-2022-unmaintained/blob/develop/src/main/java/com/swervedrivespecialties/swervelib/Mk4SwerveModuleHelper.java
   constexpr double kTurnPID_P = 1.0;
   constexpr double kTurnPID_I = 0.0;
   constexpr double kTurnPID_D = 0.1;
   constexpr double kTurnPID_F = 0.0;
   constexpr double kTurnCurrentLimit = 20.0;

   constexpr int kTurnEncoderResetIterations = 500;
   constexpr double kTurnEncoderResetMaxV = 0.5;  // no clue what this should be...

   // Swerve Module IDs
   constexpr int kModuleFrontRightID = 1;
   constexpr int kModuleRearRightID = 2;
   constexpr int kModuleFrontLeftID = 3;
   constexpr int kModuleRearLeftID = 4;
   // CAN IDs
   constexpr int kMotorDriveFrontRightID = 1;
   constexpr int kMotorDriveRearRightID = 4;
   constexpr int kMotorDriveFrontLeftID = 7;
   constexpr int kMotorDriveRearLeftID = 10;

   constexpr int kMotorTurnFrontRightID = 2;
   constexpr int kMotorTurnRearRightID = 5;
   constexpr int kMotorTurnFrontLeftID = 8;
   constexpr int kMotorTurnRearLeftID = 11;

   constexpr int kEncoderTurnFrontRightID = 3;
   constexpr int kEncoderTurnRearRightID = 6;
   constexpr int kEncoderTurnFrontLeftID = 9;
   constexpr int kEncoderTurnRearLeftID = 12;

   constexpr int kPidginID = 20;

   // TODO: calculate angle offsets of absolute encoder and update here
   constexpr double kFrontRightOffset{54.844};
   constexpr double kRearRightOffset{-154.951};
   constexpr double kFrontLeftOffset{-139.922};
   constexpr double kRearLeftOffset{103.008};

   namespace SwerveModules
   {

      //static const T_SWERVE_MODULE_PARAMS kModuleFrontRight{kModuleFrontRightID, kMotorDriveFrontRightID, kMotorTurnFrontRightID, kEncoderTurnFrontRightID, kFrontRightOffset};
      //static const T_SWERVE_MODULE_PARAMS kModuleRearRight{kModuleRearRightID, kMotorDriveRearRightID, kMotorTurnRearRightID, kEncoderTurnRearRightID, kRearRightOffset};
      //static const T_SWERVE_MODULE_PARAMS kModuleFrontLeft{kModuleFrontLeftID, kMotorDriveFrontLeftID, kMotorTurnFrontLeftID, kEncoderTurnFrontLeftID, kFrontLeftOffset};
      //static const T_SWERVE_MODULE_PARAMS kModuleRearLeft{kModuleRearLeftID, kMotorDriveRearLeftID, kMotorTurnRearLeftID, kEncoderTurnRearLeftID, kRearLeftOffset};
/*
      constexpr double kModuleFrontRightzzz[4]{kMotorDriveFrontRightID,
                                            kMotorTurnFrontRightID,
                                            kEncoderTurnFrontRightID,
                                            offsets::kFrontRightOffset};
      constexpr double kModuleRearRightzzz[4]{kMotorDriveRearRightID,
                                           kMotorTurnRearRightID,
                                           kEncoderTurnRearRightID,
                                           offsets::kRearRightOffset};
      constexpr double kModuleFrontLeftzzz[4]{kMotorDriveFrontLeftID,
                                           kMotorTurnFrontLeftID,
                                           kEncoderTurnFrontLeftID,
                                           offsets::kFrontLeftOffset};
      constexpr double kModuleRearLeftzzz[4]{kMotorDriveRearLeftID,
                                          kMotorTurnRearLeftID,
                                          kEncoderTurnRearLeftID,
                                          offsets::kRearLeftOffset};
   */
   }
   // TODO:
   namespace calculations
   {
      constexpr auto kFinalDriveRatio{6.75 * 360_deg};
      constexpr units::length::inch_t kWheelCircumference = {2 * M_PI * 3.8_in / 2};

      constexpr auto kModuleMaxSpeed{16.3_fps};
      constexpr auto kChassisMaxSpeed{16.3_fps};

      constexpr auto kModuleMaxAngularVelocity{M_PI * 1_rad_per_s};           // radians per second
      constexpr auto kModuleMaxAngularAcceleration{M_PI * 2_rad_per_s / 1_s}; // radians per second^2

      constexpr double kMotorMaxOutput = 0.5;
      constexpr double kMotorDeadband = 0.1;
   }
}
