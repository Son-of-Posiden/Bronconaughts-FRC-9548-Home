// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mathExtras;
import frc.robot.Constants;

public class LinearSlideSubsystem extends SubsystemBase {
  /** Creates a new LinearSlideSubsystem. */
  SparkMax slideMotor = new SparkMax(Constants.LinearSlide.slideMotorID, MotorType.kBrushless);

  PIDController pid = new PIDController(Constants.LinearSlide.kp, Constants.LinearSlide.ki, Constants.LinearSlide.kd);

  double slideSetpoint, slideCurrent = 0.0;

  public LinearSlideSubsystem() {
    slideSetpoint = 0.0;
    slideCurrent = 0.0;
  }

  public void setSetpoint(double setpoint) {
    slideSetpoint = mathExtras.codeStop(setpoint, Constants.LinearSlide.minHight, Constants.LinearSlide.maxHight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    slideCurrent = slideMotor.getEncoder().getPosition() * Constants.LinearSlide.encoderToMetersCoefficent;
    slideMotor.set(pid.calculate(slideCurrent, slideSetpoint));
  }

  public double getHight() {
    return (slideMotor.getEncoder().getPosition() * Constants.LinearSlide.encoderToMetersCoefficent) + Constants.LinearSlide.slideHightFromFloor;
  }
}
