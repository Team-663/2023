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
#include <frc2/command/CommandPtr.h>

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
   void UpdateElevatorSetpoint(double target);
   void IncrementElevatorSetpoint(double inc);

   void SetElevatorSpeedManual(double speed);
   double GetElevatorEncAbsolute();
   void ResetElevatorEncoder();
   void Periodic() override;

   frc2::CommandPtr ResetElevatorEncoderCommand();

 private:
   frc::DoubleSolenoid m_extendoArm{frc::PneumaticsModuleType::CTREPCM, kArmSolenoid1, kArmSolenoid2};
   frc::DoubleSolenoid m_claw{frc::PneumaticsModuleType::CTREPCM, kClawSolenoid1, kClawSolenoid2};

   WPI_TalonSRX m_elevator{kElevator_CANID};
   frc::DigitalInput m_elevatorLimDown{kElevatorLimitDown_DIOPIN};
   frc::DigitalInput m_elevatorLimUp{kElevatorLimitUp_DIOPIN};

  

   double m_elevatorSetpoint;
   double m_elevatorError;
   bool m_elevatorUsePid;
   double m_elevatorSpeedManual;

   double m_wristSetpoint;
   double m_wristError;

   void InitElevatorPID();
   void InitWristPID();
   void InitMotors();
   void DisplayValues();

   //frc::DutyCycleEncoder m_elevatorEnc{elevator_DIOPIN}; // Test out rev through bore encoder 
   //Experimental:
};
