// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Camera.h"

Camera::Camera()
{
   
}

// This method will be called once per scheduler run
void Camera::Periodic() 
{
   std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
   m_numTargets = table->GetNumber("tv",0.0);
   m_targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
   m_targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);

   frc::SmartDashboard::PutNumber("Cam Has Target", m_numTargets);
   frc::SmartDashboard::PutNumber("Cam X", m_targetOffsetAngle_Horizontal);
   frc::SmartDashboard::PutNumber("Cam Y", m_targetOffsetAngle_Vertical);
}

double Camera::GetTargetXOffSet()
{
   return m_targetOffsetAngle_Horizontal;
}

double Camera::GetTargetYOffSet()
{
   return m_targetOffsetAngle_Vertical;
}

double Camera::GetCameraValidTargets()
{
   return m_numTargets;
}
