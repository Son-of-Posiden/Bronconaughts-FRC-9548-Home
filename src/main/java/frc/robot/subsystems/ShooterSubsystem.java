// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  SparkMax leftMotor = new SparkMax(Constants.Shooter.leftMotorID, MotorType.kBrushless);
  SparkMax rightMotor  = new SparkMax(Constants.Shooter.rightMotorID, MotorType.kBrushless);

  public ShooterSubsystem() {
    
  }

  public void setLeftMotor(double speed) {
    leftMotor.set(speed);
  }

  public void setRightMotor(double speed) {
    rightMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
