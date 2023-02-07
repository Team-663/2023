// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

Arm::Arm()
{
   m_elevatorEnc.SetDistancePerRotation(100.0);
}

// This method will be called once per schedule
void Arm::Extend()
{
   m_extendoArm.Set(frc::DoubleSolenoid::kForward);
}
void Arm::Retract()
{
   m_extendoArm.Set(frc::DoubleSolenoid::kReverse);
}
void Arm::OpenClaw()
{
   m_claw.Set(frc::DoubleSolenoid::kForward);
   frc::SmartDashboard::PutString("Claw State", "Open");
}
void Arm::CloseClaw()
{
   m_claw.Set(frc::DoubleSolenoid::kReverse);
   frc::SmartDashboard::PutString("Claw State", "Closed");
}

bool Arm::IsElevatorAtUpPosition()
{
   return false;
}

bool Arm::IsElevatorAtDownPosition()
{
   return false;
}
bool Arm::IsElevatorAtSetpoint()
{
   return false;
}
void Arm::UpdateElevatorSetpoint(double target)
{
   //m_shooterElevatorSetpoint = target;
}
void Arm::IncrementElevatorSetpoint(double inc)
{
    //double newSet = m_ElevatorSetpoint + inc;
    //if (newSet > kElevatorMaxUp)
    //    newSet = kElevatorMaxUp;
    //if (newSet < kElevatorDownPosition)
    //    newSet = kElevatorDownPosition;
    
    //m_shooterElevatorSetpoint = newSet;
}
void Arm::SetElevatorSpeedManual(double speed)
{
    //m_ElevatorManualSpeed = speed * kElevatorPowerLimit;
}
void Arm::DisplayValues()
{

   frc::SmartDashboard::PutNumber("Elevator Enc Raw", m_elevatorEnc.GetDistance());
}

double Arm::GetElevatorEncAbsolute()
{
   return m_elevatorEnc.GetAbsolutePosition() / M_PI * 2.0;
}

void Arm::ResetElevatorEncoder()
{
   m_elevatorEnc.Reset();
}

frc2::CommandPtr Arm::ResetElevatorEncoderCommand() {
  return this->RunOnce(
      [this] { m_elevatorEnc.Reset(); });
}

void Arm::Periodic() 
{
   DisplayValues();

}
