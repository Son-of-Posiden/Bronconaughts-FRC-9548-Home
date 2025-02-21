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

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  SparkMax armMotor = new SparkMax(15, MotorType.kBrushless);;

  PIDController pid = new PIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd);

  public double angleSetpoint, angleCurrent = 0.0;

  public ArmSubsystem() {
    angleSetpoint = 0.0;
    angleCurrent = 0.0;
  }

  public void setSetpoint(double setpoint) {
    angleSetpoint = mathExtras.codeStop(setpoint, Constants.Arm.minAngle, Constants.Arm.maxAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    angleCurrent = armMotor.getEncoder().getPosition() * Constants.Arm.encoderToAngleCoefficent;
    armMotor.set(pid.calculate(angleCurrent, angleSetpoint));
  }

  public double getCurrentAngle() {
    return angleCurrent;
  }
}
