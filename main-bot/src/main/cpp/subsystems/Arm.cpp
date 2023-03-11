// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

Arm::Arm()
{
   InitElevatorPID();
   InitWristPID();
   InitMotors();
   CloseClaw();
   Retract();
}

void Arm::ResetElevatorEncoder()
{
   m_elevator.GetSensorCollection().SetAnalogPosition(0, kTimeoutMs);
   m_elevator.GetSensorCollection().SetQuadraturePosition(0, kTimeoutMs);
}

void Arm::ResetWristEncoder()
{
   m_wristEnc.SetZeroOffset(m_wristEnc.GetPosition() / kWristPosConversion);
}
void Arm::SetWristSetpointHere()
{
   m_wristSetpoint = m_wristEnc.GetPosition();
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
   /* TODO: enable*/
   // if (!m_isWristInsideFrame)
   m_claw.Set(frc::DoubleSolenoid::kForward);
   m_isClawOpen = true;
}
void Arm::CloseClaw()
{
   m_claw.Set(frc::DoubleSolenoid::kReverse);
   m_isClawOpen = false;
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
   return m_isElevatorAtSetpoint;
}

bool Arm::IsElevatorAtThisPosition(double pos)
{
   return (fabs(pos - m_elevator.GetSelectedSensorPosition()) < (kElevatorAllowedError+kElevatorExtraErrorMargin) ? true : false);
}

void Arm::UpdateElevatorSetpoint(double target)
{
   m_elevatorSetpoint = target;
}
void Arm::IncrementElevatorSetpoint(double inc)
{
   // double newSet = m_ElevatorSetpoint + inc;
   // if (newSet > kElevatorMaxUp)
   //     newSet = kElevatorMaxUp;
   // if (newSet < kElevatorDownPosition)
   //     newSet = kElevatorDownPosition;

   // m_shooterElevatorSetpoint = newSet;
}
void Arm::SetElevatorSpeedManual(double speed)
{
   double safetyFactor = m_elevatorMotorMult;

   /*
   if (m_isClawOpen) // if we hit in frame threshold, only allow down movement
   {
      if (speed < 0.0) // TODO check sign of this
      {
         safetyFactor = 0.0;
      }
   }
   */
   m_elevatorSpeedManual = safetyFactor * speed * kElevatorPowerLimit;
}

frc2::CommandPtr Arm::ResetElevatorEncoderCommand()
{
   return this->RunOnce(
       [this]
       {
          ResetElevatorEncoder();
       });
}

frc2::CommandPtr Arm::SetElevatorPositionCmd(double setpoint)
{
   return frc2::CommandPtr(
            frc2::FunctionalCommand(
            // Update setpoint once
            [this, setpoint] { this->UpdateElevatorSetpoint(setpoint); },
            // no execute loop, arm subsystem calculates error
            [this] { },
            // No end of command function
            [this](bool interrupted) { },
            // command finishes when elevator within error margin
            [this, setpoint] {return this->IsElevatorAtThisPosition(setpoint);},
            // Requires the arm
            {this})
   );
}

frc2::CommandPtr Arm::SetWristPositionCmd(double setpoint)
{
   return frc2::CommandPtr(
            frc2::FunctionalCommand(
            // Update setpoint once
            [this, setpoint] { this->UpdateWristSetpoint(setpoint); },
            // no execute loop, arm subsystem calculates error
            [this] { },
            // No end of command function
            [this](bool interrupted) { },
            // command finishes when elevator within error margin
            [this] {return this->IsWristAtSetpoint();},
            // Requires the arm
            {this})
   );
}

frc2::CommandPtr Arm::OpenClawCmd()
{
   return this->RunOnce([this] { this->OpenClaw(); });
}

frc2::CommandPtr Arm::CloseClawCmd()
{
   return this->RunOnce([this] { this->CloseClaw(); });
}

frc2::CommandPtr Arm::ExtendCmd()
{
   return this->RunOnce([this] { this->Extend(); });
}

frc2::CommandPtr Arm::RetractCmd()
{
   return this->RunOnce([this] { this->Retract(); });
}

#define USE_ELEVATOR_LIMIT_SWITCHES 1
// This method will be called once per scheduler run
void Arm::Periodic()
{
   m_isWristInsideFrame = (m_wristEnc.GetPosition() < kWristInFrameAngle ? true : false);
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

   if (!m_elevatorPIDActive)
   {
      if ((m_elevatorSpeedManual > 0.05 || m_elevatorSpeedManual < -0.05) && moveElevatorAllowed)
      {
         m_elevator.Set(ControlMode::PercentOutput, m_elevatorSpeedManual);
      }
      else
      {
         m_elevator.Set(0.0);
      }
   }
   else
   {
      m_elevator.Set(ControlMode::Position, m_elevatorSetpoint);
      m_isElevatorAtSetpoint = (fabs(m_elevator.GetClosedLoopError()) <= (kElevatorAllowedError+kElevatorExtraErrorMargin) ? true : false);
   }
   frc::SmartDashboard::PutBoolean("Elevator can Move?", moveElevatorAllowed);

   m_wristError = m_wristEnc.GetPosition() - m_wristSetpoint;
   m_isWristAtSetpoint = (fabs(m_wristError) <= kWristAllowedError ? true : false);

   m_wristPIDOutput = std::clamp(m_wristPID2.Calculate(m_wristEnc.GetPosition(), m_wristSetpoint), -0.5, 0.35);
   //m_wrist.Set(m_wristSpeed);
   //m_wrist.Set(m_wristPIDOutput);
   m_wristPID.SetReference(m_wristSetpoint, rev::CANSparkMax::ControlType::kPosition);
}

void Arm::ElevatorSetPIDState(bool enabled)
{
   m_elevatorPIDActive = enabled;
}

void Arm::SetWristSpeed(double val)
{
   m_wristSpeed = val;
}

// Takes scaled/checked input
void Arm::IncrementWristSetpoint(double incrVal)
{
   m_wristSetpoint += incrVal;
}

void Arm::UpdateWristSetpoint(double val)
{
   m_wristSetpoint = val;
}

bool Arm::IsWristAtSetpoint()
{
   return m_isWristAtSetpoint;
}

bool Arm::IsWristAtThisPosition(double pos)
{
   return (fabs(pos - m_wristEnc.GetPosition()) <= kWristAllowedError ? true : false);
}


// Returns TRUE if wrist is <90deg angle
// Uset to prevent both opening claw inside frame
// and rotating wrist when claw is open

void Arm::DisplayValues()
{

   frc::SmartDashboard::PutNumber("Elevator Enc Raw", m_elevator.GetSelectedSensorPosition());
   //frc::SmartDashboard::PutNumber("Elevator Manual Speed", m_elevatorSpeedManual);

   frc::SmartDashboard::PutNumber("Elevator Set", m_elevatorSetpoint);
   frc::SmartDashboard::PutNumber("Elevator Err", m_elevator.GetClosedLoopError());
   frc::SmartDashboard::PutNumber("Elv AMPS", m_elevator.GetOutputCurrent());

   frc::SmartDashboard::PutNumber("Wrist Set", m_wristSetpoint);
   //frc::SmartDashboard::PutNumber("Wrist Err", m_wristError);
   //frc::SmartDashboard::PutNumber("Wrist Speed", m_wristSpeed);
   //frc::SmartDashboard::PutNumber("Wrist EncRaw", m_wristEnc.GetPosition());
   frc::SmartDashboard::PutBoolean("Wrist at Target?", m_isWristAtSetpoint);
   //frc::SmartDashboard::PutNumber("Wrist PID Out", m_wristPIDOutput);
   frc::SmartDashboard::PutNumber("Wrist Amps", m_wrist.GetOutputCurrent());

   frc::SmartDashboard::PutBoolean("Claw Open?", m_isClawOpen);
   //frc::SmartDashboard::PutBoolean("Elev Lim Top", m_elevatorLimUp.Get());
   //frc::SmartDashboard::PutBoolean("Elev Lim Bottom", m_elevatorLimDown.Get());
   //frc::SmartDashboard::PutBoolean("ElevPid Active?", m_elevatorPIDActive);
   frc::SmartDashboard::PutBoolean("Elev at Target?", m_isElevatorAtSetpoint);
}

void Arm::InitElevatorPID()
{

   m_elevator.ConfigFactoryDefault();

   m_elevator.ConfigSelectedFeedbackSensor(
       FeedbackDevice::CTRE_MagEncoder_Relative, kElevatorPIDID, kTimeoutMs);
   m_elevator.SetSensorPhase(true);
   m_elevator.ConfigAllowableClosedloopError(kElevatorPIDID, kElevatorAllowedError, kTimeoutMs);

   m_elevator.ConfigClosedLoopPeakOutput(kElevatorPIDID, kElevatorPowerLimit, kTimeoutMs);
   m_elevator.Config_kP(kElevatorPIDID, kElevator_P, kTimeoutMs);
   m_elevator.Config_kI(kElevatorPIDID, kElevator_I, kTimeoutMs);
   m_elevator.Config_kD(kElevatorPIDID, kElevator_D, kTimeoutMs);
   m_elevator.Config_kF(kElevatorPIDID, kElevator_F, kTimeoutMs);

   // m_elevator.ConfigForwardLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource::)

   m_elevator.ConfigForwardSoftLimitThreshold(-20000.0, kTimeoutMs);
   m_elevator.ConfigForwardSoftLimitEnable(false);
   m_elevator.ConfigReverseSoftLimitThreshold(-20000.0, kTimeoutMs);
   m_elevator.ConfigReverseSoftLimitEnable(false);

   //m_elevatorSetpoint = m_elevator.GetSelectedSensorPosition();
   ElevatorSetPIDState(true);
}

void Arm::InitWristPID()
{
   m_wristPID.SetP(kWrist_P);
   m_wristPID.SetI(kWrist_I);
   m_wristPID.SetD(kWrist_D);
   m_wristPID.SetFF(kWrist_F);
   m_wristPID.SetOutputRange(kWristPIDMinOutput, kWristPIDMaxOutput);
   m_wristPID.SetFeedbackDevice(m_wristEnc);
   
   m_wrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
   m_wrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, kWristSoftLimForward);
   m_wrist.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
   m_wrist.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, kWristSoftLimReverse);
}

void Arm::InitMotors()
{
   m_wrist.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
   m_wrist.SetInverted(true); // TODO: update
   m_wristEnc.SetPositionConversionFactor(1.0);
   ResetWristEncoder();
   //m_wristSetpoint = m_wristEnc.GetPosition();


   m_elevator.SetNeutralMode(motorcontrol::NeutralMode::Brake);
   m_elevator.SetInverted(kElevatorIsMotorInverted); // TODO: update
   if (kElevatorIsMotorInverted)
      m_elevatorMotorMult = -1.0;
   else
      m_elevatorMotorMult = 1.0;

   if (kElevatorIsMotorInverted)
   {
      m_elevator.ConfigPeakOutputReverse(kElevatorPowerLimit);
      m_elevator.ConfigPeakOutputForward(-kElevatorPowerLimitDown);
   }
   else
   {
      m_elevator.ConfigPeakOutputReverse(-kElevatorPowerLimitDown);
      m_elevator.ConfigPeakOutputForward(kElevatorPowerLimit);
   }

   // Current limit seemed to prevent arm from moving fast enough
   // m_intakeArm.ConfigPeakCurrentLimit(kIntakeArmCurrentLimit);
   m_elevator.EnableCurrentLimit(false);
}