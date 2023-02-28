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

void DriveTrain::GyroResetHeading()
{
   m_pigeon.SetYawToCompass();

   // SetFusedHeading(0.0);
}

void DriveTrain::GyroSetTargetAngle(double tgtAngle)
{
   m_tgtAngle = tgtAngle;
}

double DriveTrain::GyroGetPitch()
{
   return m_pigeon.GetPitch();
}

void DriveTrain::GyroTurnToTargetAngle()
{
   m_gyroTurnError = m_tgtAngle - m_pigeon.GetYaw();

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

void DriveTrain::GyroDriveStraight(double speed, double angle)
{
   double turn = kGyroDriveKP * -m_pigeon.GetYaw();
   if (speed > 0.0)
   {
      m_drive.ArcadeDrive(speed, turn, false);
   }

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
   int sign = 1;
   if (m_driveError < 0)
      sign = -1;
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
   //m_drive.ArcadeDrive(driveAutoOutput, 0.0);
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

frc2::CommandPtr DriveTrain::DriveStraightCmd(double dist, double timeout)
{
   return frc2::CommandPtr(
            frc2::FunctionalCommand(
            // Update setpoint once
            [this, dist] {
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

// sign of maxSpeed determines if we are starting backwards on the scale (does this matter?)
frc2::CommandPtr DriveTrain::BalanceOnRampCmd(double maxSpeed)
{
   return frc2::CommandPtr(
            frc2::FunctionalCommand(
            [this] {},
            [this, maxSpeed] 
            {
               double speed = maxSpeed * kAutoBalnace_kP * (this->m_pigeon.GetRoll());
               this->m_drive.TankDrive(-speed, -speed);
               frc::SmartDashboard::PutNumber("Auto balance output", speed);
            },
            [this](bool interrupted)
            { 
               this->Stop();
            },
            // command finishes when elevator within error margin
            [this] {return m_isRobotBalanced;},
            // Requires the arm
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
   frc::SmartDashboard::PutNumber("Gyro Yaw:", m_pigeon.GetYaw());
   frc::SmartDashboard::PutNumber("Gyro Pitch:", m_pigeon.GetPitch());
   frc::SmartDashboard::PutNumber("Gyro Roll:", m_pigeon.GetRoll());
   frc::SmartDashboard::PutNumber("Tank L", m_driveLVal);
   frc::SmartDashboard::PutNumber("Tank R", m_driveRVal);

   frc::SmartDashboard::PutNumber("Drive Setpoint", m_driveSetpoint);
   frc::SmartDashboard::PutNumber("Drive Encoder", GetDriveEncoderValue());
   frc::SmartDashboard::PutNumber("Drive Error", m_driveError);
   frc::SmartDashboard::PutNumber("Drive Encoder Speed", m_encoderVelocity);
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
