// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"


Arm::Arm() 
{
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
   m_elevatorSpeedManual = speed * kElevatorPowerLimit;
}

frc2::CommandPtr Arm::ResetElevatorEncoderCommand()
{
   return this->RunOnce(
      [this] 
      {
         m_elevator.GetSensorCollection().SetAnalogPosition(0, kTimeoutMs);
         m_elevator.GetSensorCollection().SetQuadraturePosition(0, kTimeoutMs);
      });
}


// This method will be called once per scheduler run
void Arm::Periodic() 
{
   bool moveElevatorAllowed = true;
   DisplayValues();
#ifdef USE_ELEVATOR_LIMIT_SWITCHES
   if (m_elevatorLimDown.Get() == 0)
   {
      if (m_elevatorSpeedManual < 0.0)
         moveElevatorAllowed = false;
   }

   if (m_elevatorLimUp.Get() == 0)
   {
      if (m_elevatorSpeedManual > 0.0)
         moveElevatorAllowed = false;
   }

#endif

   if ( (m_elevatorSpeedManual > 0.05 || m_elevatorSpeedManual < -0.05) && moveElevatorAllowed)
   {
      m_elevator.Set(ControlMode::PercentOutput, m_elevatorSpeedManual);
   }
   else
   {
      m_elevator.Set(0.0);
   }
   frc::SmartDashboard::PutBoolean("Elevator can Move?", moveElevatorAllowed);
   //m_elevator.Set(ControlMode::Position, m_elevatorSetpoint);

}

void Arm::DisplayValues()
{

   frc::SmartDashboard::PutNumber("Elevator Enc Raw", m_elevator.GetSelectedSensorPosition());
   frc::SmartDashboard::PutNumber("Elevator Manual Speed", m_elevatorSpeedManual);

   frc::SmartDashboard::PutNumber("Elevator Set", m_elevatorSetpoint);
   frc::SmartDashboard::PutNumber("Elevator Err", m_elevatorError);
   
   
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
}

void Arm::InitWristPID()
{
   
}

void Arm::InitMotors()
{
   m_elevator.SetNeutralMode(motorcontrol::NeutralMode::Brake);
   

   m_elevator.SetInverted(false); // TODO: update


   m_elevator.ConfigPeakOutputReverse(-kElevatorPowerLimit);
   m_elevator.ConfigPeakOutputForward(kElevatorPowerLimit);

   
   
   // Current limit seemed to prevent arm from moving fast enough
   //m_intakeArm.ConfigPeakCurrentLimit(kIntakeArmCurrentLimit);
   m_elevator.EnableCurrentLimit(false);

}