// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

Arm::Arm()
{


   //m_WristEnc.SetDistancePerRotation(kElevatorEncDistPerRev);

   InitElevatorPID();




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
   m_elevatorSetpoint = target;
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
   m_elevatorManualSpeed = speed * kElevatorPowerLimit;
}
void Arm::DisplayValues()
{

   frc::SmartDashboard::PutNumber("Elevator Enc Raw", m_elevatorEnc.GetDistance());
   frc::SmartDashboard::PutNumber("Elevator Manual Speed", m_elevatorManualSpeed);
   
   
}

double Arm::GetElevatorEncAbsolute()
{
   return m_elevatorEnc.GetAbsolutePosition() / M_PI * 2.0;
}

void Arm::ResetElevatorEncoder()
{
   m_elevatorEnc.Reset();
}

// WARNING Trevor Nathan work below

bool Arm::IsWristAtUpPosition()
{
   return false;
}

bool Arm::IsWristAtDownPosition()
{
   return false;
}

bool Arm::IsWristAtSetpoint()
{
   return false;
}

void Arm::UpdateWristSetpoint(double target)
{
   m_wristSetpoint = target;
}

void Arm::ResetWristEncoder()
{
   m_wristEnc.Reset();
}

double Arm::GetWristEncAbsolute()
{
   return m_wristEnc.GetAbsolutePosition() / M_PI * 2;
}


frc2::CommandPtr Arm::ResetElevatorEncoderCommand() {
  return this->RunOnce(
      [this] { m_elevatorEnc.Reset(); });
}

//frc2::CommandPtr Arm::ElevatorManualSpeed(double s)
//{
   //return this->RunOnce(
   //   [this] {SetElevatorSpeedManual(s); }
   //   );
//}

void Arm::Periodic() 
{
   bool moveElevatorAllowed = true;
   DisplayValues();

   if (m_elevatorLimDown.Get() == 0)
   {
      if (m_elevatorManualSpeed < 0.0)
         moveElevatorAllowed = false;
   }

   if (m_elevatorLimUp.Get() == 0)
   {
      if (m_elevatorManualSpeed > 0.0)
         moveElevatorAllowed = false;
   }


   if ( (m_elevatorManualSpeed > 0.05 || m_elevatorManualSpeed < -0.05) && moveElevatorAllowed)
   {
      m_elevator.Set(ControlMode::PercentOutput, m_elevatorManualSpeed);
   }
   else
   {
      m_elevator.Set(0.0);
   }
   frc::SmartDashboard::PutBoolean("Elevator can Move?", moveElevatorAllowed);
   //m_elevator.Set(ControlMode::Position, m_elevatorSetpoint);

}

void Arm::InitElevatorPID()
{

   m_elevator.ConfigFactoryDefault();

   m_elevator.ConfigSelectedFeedbackSensor(
        FeedbackDevice::CTRE_MagEncoder_Relative, kElevatorPIDID, kTimeoutMs);
   m_elevator.SetSensorPhase(false);
   m_elevator.ConfigAllowableClosedloopError(kElevatorPIDID, kElevatorAllowedError, kTimeoutMs);

   m_elevator.ConfigClosedLoopPeakOutput(kElevatorPIDID, kElevatorPowerLimit, kTimeoutMs);
	m_elevator.Config_kP(kElevatorPIDID, kElevator_P, kTimeoutMs);
	m_elevator.Config_kI(kElevatorPIDID, kElevator_I, kTimeoutMs);
	m_elevator.Config_kD(kElevatorPIDID, kElevator_D, kTimeoutMs);
   m_elevator.Config_kF(kElevatorPIDID, kElevator_F, kTimeoutMs);

    //m_elevator.ConfigReverseSoftLimitThreshold(kIntakeArmDownPosition);
    //m_elevator.ConfigReverseSoftLimitEnable(true);

    //m_elevator.ConfigForwardSoftLimitThreshold(0.0);
    //m_elevator.ConfigForwardSoftLimitEnable(true);
}
