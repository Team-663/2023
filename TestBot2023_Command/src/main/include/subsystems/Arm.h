// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Compressor.h>
#include <Constants.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Encoder.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/DigitalInput.h>
#include "math.h"
#include <frc2/command/CommandPtr.h>
#include <rev/CANSparkMax.h>
using namespace ArmConstants;

class Arm : public frc2::SubsystemBase {
 public:
  Arm();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
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

   bool IsWristAtUpPosition(); //false
  bool IsWristAtDownPosition(); //false
  bool IsWristAtSetpoint(); //false
  void UpdateWristSetpoint(double target); //works
  //void IncrementWristSetpoint(double inc); //seems useless

  //void SetWristSpeedManual(double speed); //elevator version is commented out
  double GetWristEncAbsolute();
  void ResetWristEncoder();

  frc2::CommandPtr ResetElevatorEncoderCommand();
  //frc2::CommandPtr ElevatorManualSpeed(double s);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::Compressor m_pcmComp{compressor_CANID, frc::PneumaticsModuleType::CTREPCM};
  frc::DoubleSolenoid m_extendoArm{frc::PneumaticsModuleType::CTREPCM, extendoArm_solenoid1, extendoArm_solenoid2};
  frc::DoubleSolenoid m_claw{frc::PneumaticsModuleType::CTREPCM, claw_solenoid1, claw_solenoid2};
  rev::CANSparkMax m_wristbow{wristbow_CANID};
  WPI_TalonSRX m_elevator{elevator_CANID};
  frc::DigitalInput m_elevatorLimDown{kElevatorLimitDown_DIOPIN};
  frc::DigitalInput m_elevatorLimUp{kElevatorLimitUp_DIOPIN};
  //frc::DutyCycleEncoder m_elevatorEnc{elevator_DIOPIN}; // Test out rev through bore encoder 
  frc::PIDController m_elevatorPID;
  //Experimental:
  frc::DutyCycleEncoder m_wristEnc{wrist_DIOPIN};

  double m_elevatorSetpoint;
  double m_elevatorError;

  double m_wristSetpoint;
  double m_wristError;

  double m_elevatorManualSpeed;

   void InitializeArmPid();
   void DisplayValues();

  // horizontal piston
  //frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};
  // vertical elevator
  // left gripper
  // right gripper
  // wristbow piston
  //frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};

};
