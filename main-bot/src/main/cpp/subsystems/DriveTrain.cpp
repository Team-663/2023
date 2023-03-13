// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"

using namespace DriveTrainConstants;

DriveTrain::DriveTrain() //: 
                           //m_driveMotorL1{kDriveNEOL1_CANID, rev::CANSparkMax::MotorType::kBrushless},
                          // m_driveMotorL2{kDriveNEOL2_CANID, rev::CANSparkMax::MotorType::kBrushless},
                          // m_driveMotorL3{kDriveNEOL3_CANID, rev::CANSparkMax::MotorType::kBrushless},
                           //m_driveMotorR1{kDriveNEOR1_CANID, rev::CANSparkMax::MotorType::kBrushless},
                           //m_driveMotorR2{kDriveNEOR2_CANID, rev::CANSparkMax::MotorType::kBrushless},
                           //m_driveMotorR3{kDriveNEOR3_CANID, rev::CANSparkMax::MotorType::kBrushless},

                           //m_pigeon{kPigin_CANID}
{
   m_leftMotors.SetInverted(true);
   m_DriveL1encoder.SetPositionConversionFactor(kEncTicsPerInch);
   m_driveSetpoint = 0.0;
   GyroSetTargetAngleHere();
}

// This method will be called once per scheduler run
void DriveTrain::TankDrive(double l, double r)
{

   m_driveLVal = l;
   m_driveRVal = r;

   m_drive.TankDrive(l, r, true);
}

void DriveTrain::Stop()
{
   TankDrive(0, 0);
}

void DriveTrain::ToggleMotorNeutralMode()
{
   if (m_isBrakeMode)
   {
      SetMotorMode(false);
   }
   else
   {
      SetMotorMode(true);
   }
}

void DriveTrain::SetMotorMode(bool brake)
{
   if (brake)
   {
      m_driveMotorL1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      m_driveMotorL2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      m_driveMotorL3.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

      m_driveMotorR1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      m_driveMotorR2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      m_driveMotorR3.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      m_isBrakeMode = true;
   }
   else
   {
      m_driveMotorL1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
      m_driveMotorL2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
      m_driveMotorL3.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

      m_driveMotorR1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
      m_driveMotorR2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
      m_driveMotorR3.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
      m_isBrakeMode = false;
   }

}

void DriveTrain::GyroResetHeading()
{
   m_pigeon.SetYawToCompass();

   // SetFusedHeading(0.0);
}

void DriveTrain::GyroSetTargetAngleHere()
{
   GyroSetTargetAngle(m_pigeon.GetYaw());
}

void DriveTrain::GyroSetTargetAngle(double tgtAngle)
{
   m_tgtAngle = tgtAngle;
}

void DriveTrain::GyroSetTargetAngleOffset(double offset)
{
   m_tgtAngle += offset;
}

double DriveTrain::GyroGetPitch()
{
   return m_pigeon.GetPitch();
}

double DriveTrain::GyroGetRoll(bool inverse)
{
   if (inverse)
      return -m_pigeon.GetRoll();
   else
      return m_pigeon.GetRoll();
}

void DriveTrain::GyroTurnToTargetAngle()
{
   

   if (fabs(m_gyroTurnError) > kGyroRotateAngleBand)
   {
      double diffSpeed = fabs(m_gyroTurnError) * kGyroRotateKP;
      diffSpeed = (diffSpeed > kGyroMaxTurnRate ? kGyroMaxTurnRate : diffSpeed);
      double mult = 1.0;         // default to positive 1 mult
      if (m_gyroTurnError < 0.0) // Negative multiplier for negative error
      {
         mult = -1.0;
      }

      m_drive.TankDrive(mult * diffSpeed, -mult * diffSpeed, false);
      // m_diffDrive.ArcadeDrive(0.0, m_gyroTurnError * kGyroRotateKP, false);
   }
   else
   {
      Stop();
   }
}

bool DriveTrain::GyroIsAtHeading(double band)
{
   if (fabs(m_gyroTurnError) > band)
   {
      return false;
   }
   else
   {
      return true;
   }
}

void DriveTrain::GyroDriveStraight(double speed)
{
   double turn = kGyroDriveKP * -m_gyroTurnError;
   //kGyroDriveStraightMaxSpeed
   //if (speed > 0.0)
   //{
      m_drive.ArcadeDrive(speed, turn, false);
   //}

   // TODO : do we need backwards??
}

double DriveTrain::GetDriveEncoderValue()
{
   return m_encoderPosition;
}

double DriveTrain::GetDriveEncoderVelocity()
{
   return m_encoderVelocity;
}

void DriveTrain::DriveToSetpoint()
{
   int sign = -1;
   if (m_driveError < 0)
      sign = 1;
   double driveAutoOutput = 0;

   double errorMag = fabs(m_driveError);

   if (errorMag > kDriveAutoProportionalDist)
   {
      driveAutoOutput = sign * kDriveAutoMaxOutput;
   }
   else
   {
      driveAutoOutput = sign * (errorMag / kDriveAutoProportionalDist) * kDriveAutoMaxOutput;
   }
   // TODO: enable
   m_drive.ArcadeDrive(driveAutoOutput, 0.0);
   frc::SmartDashboard::PutNumber("Drive Auto Output:", driveAutoOutput);
}

void DriveTrain::UpdateDriveSetpoint(double dist)
{
   m_driveSetpoint = dist;
}

bool DriveTrain::IsDriveAtSetpoint()
{
   return m_isDriveAtSetpoint;
}

bool DriveTrain::IsRobotBalanced()
{
   return m_isRobotBalanced;
}

void DriveTrain::ResetDriveEncoders()
{
   m_DriveL1encoder.SetPosition(0.0);
   m_DriveR1encoder.SetPosition(0.0);
}

frc2::CommandPtr DriveTrain::DriveStraightCmd(double dist, double timeout)
{
   return frc2::CommandPtr(
            frc2::FunctionalCommand(
            // Update setpoint once
            [this, dist] {
                this->ResetDriveEncoders();
                this->UpdateDriveSetpoint(dist);
                this->SetDrivetrainRamprate(kDriveRampRateAuto);
             },
            // no execute loop, arm subsystem calculates error
            [this] { this->DriveToSetpoint(); },
            // No end of command function
            [this](bool interrupted) {
               this->Stop();
               this->SetDrivetrainRamprate(kDriveRampRateTeleop);
               },
            // command finishes when elevator within error margin
            [this] {return this->IsDriveAtSetpoint();},
            // Requires the arm
            {this})
   );
}

frc2::CommandPtr DriveTrain::RotateToAngleCmd(double angle)
{
   return frc2::CommandPtr(
      frc2::FunctionalCommand(
         [this, angle] {
               this->GyroSetTargetAngleHere();
               this->GyroSetTargetAngleOffset(angle);
            },
            [this] 
            {
               this->GyroTurnToTargetAngle();
            },
            [this](bool interrupted) {this->Stop();},
            [this] {return this->GyroIsAtHeading(kGyroRotateAngleBand);},
            {this})
   );
}

void DriveTrain::ResetRobotMaxRoll()
{
   m_maxRoll = 0.0;
}

bool DriveTrain::HasRobotGoneUpRamp()
{
   m_maxRoll = (m_maxRoll > GyroGetRoll(true) ? m_maxRoll : GyroGetRoll(true)); 
   if (m_maxRoll > kAutoRobotOnRampRollAngle)
      m_robotOnRamp = true;
   else
      m_robotOnRamp = false;

   return m_robotOnRamp;
}

// sign of maxSpeed determines if we are starting backwards on the scale (does this matter?)
frc2::CommandPtr DriveTrain::BalanceOnRampCmd()
{
   return frc2::CommandPtr(
            frc2::FunctionalCommand(
            [this] {
               this->ResetDriveEncoders();
               this->UpdateDriveSetpoint(kAutoBalanceMaxDriveDistance);
               this->SetMotorMode(true);
               this->ResetRobotMaxRoll();
            },
            [this] 
            {
               this->m_drive.TankDrive(kAutoBalananceDriveSpeed, kAutoBalananceDriveSpeed, false);
               frc::SmartDashboard::PutNumber("AutoSpeed", this->m_driveMotorL1.Get());
               // TODO: use 
            },
            [this](bool interrupted)
            {
               this->Stop();
               //this->SetMotorMode(false);
            },
            // if we hit angle thold and are on our way down, stop
            [this] {
               if ( (this->HasRobotGoneUpRamp() && GyroGetRoll(true) < kAutoRobotBalanceAngleStop)
               || (this->IsDriveAtSetpoint()) ) // OR if we have messed up and drove too far just give up
                  return true;
               else 
                  return false;
               },
            {this})
   );
}

void DriveTrain::Periodic()
{
   m_encoderPosition = m_DriveL1encoder.GetPosition();
   m_encoderVelocity = m_DriveL1encoder.GetVelocity();
   m_driveError = m_driveSetpoint - m_encoderPosition;
   m_isDriveAtSetpoint = ( fabs(m_driveError) <= kDriveAutoErrorMargin ? true : false );
   m_isRobotBalanced = (   ((fabs(m_encoderVelocity) < kDriveBalanceWheelRotationMargin) 
                        && (fabs(GyroGetPitch()) < kDriveBalanceAngleMargin)) ? true : false 
                        );

   m_gyroAngle = m_pigeon.GetYaw();
   m_gyroTurnError = m_tgtAngle - m_pigeon.GetYaw();
   DisplayValues();
}

void DriveTrain::SetDrivetrainRamprate(double rate)
{
   m_driveMotorL1.SetOpenLoopRampRate(rate);
   m_driveMotorL2.SetOpenLoopRampRate(rate);
   m_driveMotorL3.SetOpenLoopRampRate(rate);
   m_driveMotorR1.SetOpenLoopRampRate(rate);
   m_driveMotorR2.SetOpenLoopRampRate(rate);
   m_driveMotorR3.SetOpenLoopRampRate(rate);
}

void DriveTrain::DisplayValues()
{
   //frc::SmartDashboard::PutNumber("Gyro angle Raw:", m_pigeon.Getgy);
   frc::SmartDashboard::PutNumber("Gyro Angle", m_gyroAngle);
   //frc::SmartDashboard::PutNumber("Gyro Pitch:", m_pigeon.GetPitch());
   frc::SmartDashboard::PutNumber("Gyro Roll:",GyroGetRoll(true));
   frc::SmartDashboard::PutNumber("Gyro Target:", m_tgtAngle);
   frc::SmartDashboard::PutNumber("Gyro Error:", m_gyroTurnError);
   frc::SmartDashboard::PutNumber("Tank L", m_driveLVal);
   frc::SmartDashboard::PutNumber("Tank R", m_driveRVal);

   frc::SmartDashboard::PutNumber("Drive Setpoint", m_driveSetpoint);
   frc::SmartDashboard::PutNumber("Drive Encoder", GetDriveEncoderValue());
   frc::SmartDashboard::PutNumber("Drive Error", m_driveError);
   frc::SmartDashboard::PutNumber("DriveLSpeed", m_encoderVelocity);
   frc::SmartDashboard::PutNumber("DriveRSpeed", m_DriveR1encoder.GetVelocity());
   if (m_isBrakeMode)
      frc::SmartDashboard::PutString("MotorIdleMode", "Brake");
   else
      frc::SmartDashboard::PutString("MotorIdleMode", "Coast");

   frc::SmartDashboard::PutBoolean("Robot On Ramp?", HasRobotGoneUpRamp());
   frc::SmartDashboard::PutNumber("Robot max roll", m_maxRoll);
   
   /*
   std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
   double targets = table->GetNumber("tv",0.0);
   double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
   double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
   frc::SmartDashboard::PutNumber("Cam Has Target", targets);
   frc::SmartDashboard::PutNumber("Cam X", targetOffsetAngle_Horizontal);
   frc::SmartDashboard::PutNumber("Cam Y", targetOffsetAngle_Vertical);
   */
}
