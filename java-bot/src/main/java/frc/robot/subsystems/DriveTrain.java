
/*
 * Mark Yarger
 * Nathan Romine
 */



package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class DriveTrain extends SubsystemBase{
   public DriveTrain(){};
   public final int kLeftMotor1Port = 1;

// CANSparkMax hello = new CANSparkMax(CANSparkMaxLowLevel.MotorType);

   private final MotorControllerGroup m_leftMotors =
   new MotorControllerGroup(
       new CANSparkMax(DriveConstants.kLeftMotor1Port),
       new CANSparkMax(DriveConstants.kLeftMotor2Port));

// The motors on the right side of the drive.
private final MotorControllerGroup m_rightMotors =
   new MotorControllerGroup(
       new CANSparkMax(DriveConstants.kRightMotor1Port),
       new CANSparkMax(DriveConstants.kRightMotor2Port));

}
