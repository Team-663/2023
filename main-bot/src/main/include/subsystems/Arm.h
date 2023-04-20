// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Encoder.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/DigitalInput.h>
#include "math.h"
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxAlternateEncoder.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <rev/SparkMaxPIDController.h>
#include <frc/controller/PIDController.h>

using namespace ArmConstants;

class Arm : public frc2::SubsystemBase {
public:
  Arm();

   void Extend();
   void Retract();
   void OpenClaw();
   void CloseClaw();
   bool IsElevatorAtUpPosition();
   bool IsElevatorAtDownPosition();
   bool IsElevatorAtSetpoint();
   bool IsElevatorAtThisPosition(double pos);
   void ElevatorSetPIDState(bool enabled);
   void UpdateElevatorSetpoint(double target);
   void IncrementElevatorSetpoint(double inc);
   void SetWristSpeed(double val);
   void IncrementWristSetpoint(double incrVal);
   void UpdateWristSetpoint(double val);
   bool IsWristAtSetpoint();
   bool IsWristAtThisPosition(double pos);
   void SetWristSetpointHere();

   void SetElevatorSpeedManual(double speed);
   double GetElevatorEncAbsolute();
   void ResetElevatorEncoder();
   void ResetWristEncoder();
   void Periodic() override;

   frc2::CommandPtr ResetElevatorEncoderCommand();
   frc2::CommandPtr SetElevatorPositionCmd(double setpoint);
   frc2::CommandPtr SetWristPositionCmd(double setpoint);
   frc2::CommandPtr OpenClawCmd();
   frc2::CommandPtr CloseClawCmd();
   frc2::CommandPtr ExtendCmd();
   frc2::CommandPtr RetractCmd();

 private:
   frc::DoubleSolenoid m_extendoArm{frc::PneumaticsModuleType::CTREPCM, kArmSolenoid1, kArmSolenoid2};
   frc::DoubleSolenoid m_claw{frc::PneumaticsModuleType::CTREPCM, kClawSolenoid1, kClawSolenoid2};
   
   rev::CANSparkMax m_wrist{kWrist_CANID, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
   rev::SparkMaxAbsoluteEncoder m_wristEnc = m_wrist.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);
   rev::SparkMaxPIDController m_wristPID = m_wrist.GetPIDController();
   frc::PIDController m_wristPID2{kWrist_P, kWrist_I, kWrist_D};

   WPI_TalonSRX m_elevator{kElevator_CANID};
   frc::DigitalInput m_elevatorLimDown{kElevatorLimitDown_DIOPIN};
   frc::DigitalInput m_elevatorLimUp{kElevatorLimitUp_DIOPIN};

   double m_elevatorSetpoint;
   double m_elevatorError;
   bool m_elevatorUsePid;
   double m_elevatorSpeedManual;
   double m_elevatorMotorMult;
   bool m_elevatorPIDActive;
   bool m_isElevatorAtSetpoint;

   double m_wristSetpoint;
   double m_wristError;
   double m_wristSpeed;
   bool m_isWristAtSetpoint;
   bool m_isWristInsideFrame;
   double m_wristPIDOutput;

   bool m_isClawOpen;
   void InitElevatorPID();
   void InitWristPID();
   void InitMotors();
   void DisplayValues();

   //frc::DutyCycleEncoder m_elevatorEnc{elevator_DIOPIN}; // Test out rev through bore encoder 
   //Experimental:
};
