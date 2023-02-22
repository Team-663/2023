// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ArmConstants;

class Wrist : public frc2::SubsystemBase {
 public:
  Wrist();
  
  void MoveWrist(double val);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  rev::CANSparkMax m_wrist{kWrist_CANID, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  double m_wristSpeed;
   //frc::DutyCycleEncoder m_wristEnc{kWrist_DIOPIN};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
