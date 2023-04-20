// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Credit to ahayden04 on github
// Adapted from https://github.com/ahayden04/swerve-falcon-2022

#include "SwerveModule.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <iostream>

SwerveModule::SwerveModule(const int module[])
    : m_motorDrive(module[0], rev::CANSparkMax::MotorType::kBrushless),
      m_motorTurn(module[1], rev::CANSparkMax::MotorType::kBrushless),
      m_encoderTurn(module[2]),
      m_encoderOffset(module[3]) 
{
    std::cout << "\nFor module #" << module[0] << ":\n";
    std::cout << m_motorDrive.GetDeviceId() << " - drive Falcon ID.\n";
    std::cout << m_motorTurn.GetDeviceId() << " - turn Falcon ID.\n";
    std::cout << m_encoderTurn.GetDeviceNumber() << " - CANCoder for steering.\n\n";
    ConfigDriveModule();
    ConfigSteerModule();
    ConfigEncoderModule();
}
void SwerveModule::ConfigSteerModule()
{

}

void SwerveModule::ConfigEncoderModule()
{

}
void SwerveModule::ConfigDriveModule()
{
   m_motorDrive.RestoreFactoryDefaults();
   // TODO: port this
   //m_motorDrive.GetPIDController().SetFeedbackDevice()
/*
   motorDrive.primaryPID.selectedFeedbackSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
   motorDrive.primaryPID.selectedFeedbackCoefficient = 1.0;
   motorDrive.closedloopRamp = 1.705000;
   motorDrive.peakOutputForward = 0.20F;
   motorDrive.peakOutputReverse = -0.20F;
   motorDrive.nominalOutputForward = 0.0;
   motorDrive.nominalOutputReverse = -0.0;
   motorDrive.neutralDeadband = 0.001;
   motorDrive.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_100Ms;
   motorDrive.slot0.kP = 0;
   motorDrive.slot0.kI = 0;
   motorDrive.slot0.kD = 0;
   motorDrive.slot0.kF = (0.5 * 1023.0) / (22100.0 * 0.5);
   // motorDrive.slot0.integralZone = 900;
   motorDrive.slot0.allowableClosedloopError = 0;
   // motorDrive.slot0.maxIntegralAccumulator = 254.000000;
   motorDrive.slot0.closedLoopPeakOutput = 1.0;
   // motorDrive.slot0.closedLoopPeriod = 33;
   motorDrive.remoteSensorClosedLoopDisableNeutralOnLOS = false;
   ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration supplyLimit(true, 23.1, 25, 1.4);
   // motorDrive.supplyCurrLimit = supplyLimit;
   ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration statorLimit(true, 12.1, 87.4, 0.4);
   // motorDrive.statorCurrLimit = statorLimit;
   motorDrive.motorCommutation = ctre::phoenix::motorcontrol::MotorCommutation::Trapezoidal;
   motorDrive.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
   */
}

/*
void swerveModule::ConfigModule(const ConfigType& type) {
    switch(type) {
        case ConfigType::motorDrive :
            m_motorDrive.ConfigFactoryDefault();
            m_motorDrive.ConfigAllSettings(m_settings.motorDrive);
            //Bevel gear on left side.
            m_motorDrive.SetInverted(ctre::phoenix::motorcontrol::TalonFXInvertType::Clockwise);
            m_motorDrive.SelectProfileSlot(0, 0);
            break;
        case ConfigType::motorTurn :
            m_motorTurn.ConfigFactoryDefault();
            m_motorTurn.ConfigAllSettings(m_settings.motorTurn);
            m_motorTurn.ConfigRemoteFeedbackFilter(m_encoderTurn.GetDeviceNumber(),
                                                   ctre::phoenix::motorcontrol::
                                                   RemoteSensorSource::RemoteSensorSource_CANCoder, 0, 50);
            m_motorTurn.SetInverted(ctre::phoenix::motorcontrol::TalonFXInvertType::CounterClockwise);
            m_motorTurn.SelectProfileSlot(0, 0);
            break;
        case ConfigType::encoderTurn :
            m_encoderTurn.ConfigFactoryDefault();
            m_encoderTurn.ConfigAllSettings(m_settings.encoderTurn);
            m_encoderTurn.ConfigMagnetOffset(m_encoderOffset);
            break;
        default :
            throw std::invalid_argument("Invalid swerveModule ConfigType");
    }
}
*/
frc::SwerveModuleState SwerveModule::GetState() {
    units::native_units_per_decisecond_t motorSpeed{m_motorDrive.GetEncoder().GetVelocity()};
    units::meters_per_second_t wheelSpeed{
        (motorSpeed * drivetrainConstants::calculations::kWheelCircumference)
        / drivetrainConstants::calculations::kFinalDriveRatio};
    return {wheelSpeed,frc::Rotation2d(frc::AngleModulus(units::degree_t(m_encoderTurn.GetPosition())))};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
    const auto state = CustomOptimize(
        referenceState,units::degree_t(m_encoderTurn.GetPosition()));
    //This returns the position in +-Cancoder units counting forever, as opposed to absolulte -180 to +180 deg.

    const auto targetWheelSpeed{state.speed};
    m_targetAngle = state.angle.Degrees().value();
    const double turnOutput = m_targetAngle * (4096.0/360.0);

    units::native_units_per_decisecond_t targetMotorSpeed{
        (targetWheelSpeed * drivetrainConstants::calculations::kFinalDriveRatio)
        / drivetrainConstants::calculations::kWheelCircumference};
    
    //m_motorDrive.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, targetMotorSpeed.value());
    //std::cout << targetMotorSpeed.value() << "-target_SPEED\n";

    //m_motorTurn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position, turnOutput);
    //std::cout << turnOutput << "u\n";
}

frc::SwerveModuleState SwerveModule::CustomOptimize(const frc::SwerveModuleState& desiredState,
                                                    const frc::Rotation2d& currentAngle) {
    auto modulusAngle{frc::AngleModulus(currentAngle.Degrees())};
    auto optAngle = desiredState.angle;
    auto optSpeed = desiredState.speed;
    
    auto difference = optAngle.Degrees() - modulusAngle;
    frc::SmartDashboard::PutNumber("Difference", difference.value());

    if (difference >= 270_deg) {
        difference = difference - 360_deg;
    } else if (difference <= -270_deg) {
        difference = difference + 360_deg;
    }

    if (units::math::abs(difference) > 90_deg) {
        optSpeed = -desiredState.speed;
        if (difference > 0_deg) {
            difference = difference - 180_deg;
        } else {
            difference = difference + 180_deg;
        }
    }
    optAngle = currentAngle.Degrees() + difference;

    frc::SmartDashboard::PutNumber("Desired Angle", optAngle.Degrees().value());
    return {optSpeed, optAngle};
}

double SwerveModule::DashboardInfo(const DataType& type) {
    switch(type) {
        case DataType::kCurrentAngle :
            return {units::degree_t(frc::AngleModulus(units::degree_t(m_encoderTurn.GetPosition()))).value()};
        /*case DataType::kCurrentVelocity :
            units::native_units_per_decisecond_t motorSpeed{m_motorDrive.GetSelectedSensorVelocity(0)};
            units::meters_per_second_t wheelSpeed{
                (motorSpeed * drivetrainConstants::calculations::kWheelCircumference)
                / drivetrainConstants::calculations::kFinalDriveRatio};
            return {wheelSpeed.value()};*/
        case DataType::kTargetAngle :
            return {m_targetAngle};
        default :
            throw std::invalid_argument("Invalid DashboardInfo DataType");
    }
}