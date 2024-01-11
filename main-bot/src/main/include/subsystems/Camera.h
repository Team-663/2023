// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

class Camera : public frc2::SubsystemBase {
 public:
  Camera();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  double GetTargetXOffSet();
  double GetTargetYOffSet();
  double GetCameraValidTargets();

 private:
   bool m_hasTarget;
   double m_numTargets;
   double m_targetOffsetAngle_Horizontal;
   double m_targetOffsetAngle_Vertical;
   double m_tx;
   double m_ty;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
