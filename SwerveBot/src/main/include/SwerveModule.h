// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/sensors/WPI_CANCoder.h>
#include <rev/CANSparkMax.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <frc/controller/PIDController.h>

#include "Constants.h"
#include "hardwareSettings.h"

//using namespace DriveTrainConstants;

struct T_SWERVE_MODULE_PARAMS
{
   int moduleId;
   int driveMotorId;
   int turnMotorId;
   int encoderId;
   double encoderOffsetDeg;
};

class SwerveModule
{
public:
   SwerveModule(int moduleId, int driveId, int turnId, int encId, double encoderOffsetDegs);
   SwerveModule(T_SWERVE_MODULE_PARAMS params);
   void InitComponents();
   // enum class ConfigType {motorDrive, motorTurn, encoderTurn};
   // void ConfigModule(const ConfigType& type);
   void ConfigDriveModule();
   void ConfigSteerModule();
   void ConfigEncoderModule();
   frc::SwerveModuleState GetState();

   void SetDesiredState(const frc::SwerveModuleState &state);

   frc::SwerveModuleState CustomOptimize(const frc::SwerveModuleState &desiredState, const frc::Rotation2d &currentAngle);

   enum class DataType
   {
      kCurrentAngle,
      kCurrentVelocity,
      kTargetAngle
   };
   double DashboardInfo(const DataType &type);

private:
   // ctre::phoenix::motorcontrol::can::WPI_TalonFX m_motorDrive;
   int m_moduleId;
   rev::CANSparkMax m_motorDrive;
   rev::CANSparkMax m_motorTurn;
   // ctre::phoenix::motorcontrol::can::WPI_TalonFX m_motorTurn;
   ctre::phoenix::sensors::WPI_CANCoder m_encoderTurn;
   ctre::phoenix::sensors::CANCoderConfiguration m_encoderTurnConfig;
   rev::SparkMaxPIDController m_drivePID = m_motorDrive.GetPIDController();
   rev::SparkMaxPIDController m_turnPID = m_motorTurn.GetPIDController();
   // frc2::PIDController m_turnPID{0.0, 0.0, 0.0};
   const double m_encoderOffset;
   double m_targetAngle;
   int m_resetIteration;
   double SetReferenceAngle(double tgtAngle);

   hardwareSettings m_settings;
};

namespace units
{
   UNIT_ADD(angle, native_unit, native_units, nu, unit<std::ratio<360, 2048>, units::degrees>) // 2048 clicks per rotation.
   UNIT_ADD(angular_velocity, native_units_per_decisecond, native_units_per_decisecond, nu_per_ds,
            compound_unit<native_units, inverse<deciseconds>>) // clicks per 100ms (standard FX output).
   // UNIT_ADD(angle, drive_gearing, drive_gearing, dratio, unit<std::ratio<27, 4>, units::degrees>)
   UNIT_ADD(length, wheel_circumference, wheel_circumferences, wcrc, unit<std::ratio<1194, 100>, units::inches>) // 3.8in diameter.
}

