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
#include "math.h"
#include <frc2/command/CommandPtr.h>
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

  frc2::CommandPtr ResetElevatorEncoderCommand();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::Compressor m_pcmComp{compressor_CANID, frc::PneumaticsModuleType::CTREPCM};
  frc::DoubleSolenoid m_extendoArm{frc::PneumaticsModuleType::CTREPCM, extendoArm_solenoid1, extendoArm_solenoid2};
  frc::DoubleSolenoid m_claw{frc::PneumaticsModuleType::CTREPCM, claw_solenoid1, claw_solenoid2};
  WPI_TalonSRX m_wristbow{wristbow_CANID};
  frc::DutyCycleEncoder m_elevatorEnc{elevator_DIOPIN}; // Test out rev through bore encoder 
  //WPI_VictorSPX m_elevator{elevator_CANID};

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
