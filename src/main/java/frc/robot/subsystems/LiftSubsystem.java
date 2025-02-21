// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
  /** Creates a new LiftSubsystem. */

  SparkMax liftMotor = new SparkMax(Constants.Lift.liftMotorID, MotorType.kBrushless);

  public LiftSubsystem() {

  }

  public void setSpeed(double speed) {
    if (Constants.Lift.minHight > liftMotor.getEncoder().getPosition() || liftMotor.getEncoder().getPosition() > Constants.Lift.maxHight) {
      speed = 0.0;
    }
    liftMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
