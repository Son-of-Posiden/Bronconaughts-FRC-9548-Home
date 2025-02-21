// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AdvancedSwerveDriveCommand extends Command {
  /** Creates a new AdvancedSwerveDriveCommand. */
  SwerveSubsystem swerve;

  double targetSpeedX, targetSpeedY, targetAngle, targetSpeed, radious, turnByAngle, currentAngle, mDegrees, startingAngle = 0.0;

  double appliedSpeed, appliedAngle = 0.0;

  boolean isCircle, isMotionTest, testsFinished = false;

  public AdvancedSwerveDriveCommand(SwerveSubsystem swerve, DoubleSupplier targetSpeedX, DoubleSupplier targetSpeedY, DoubleSupplier targetAngle) {
    this.swerve = swerve;
    this.targetSpeedX = targetSpeedX.getAsDouble();
    this.targetSpeedY = targetSpeedY.getAsDouble();
    this.targetAngle = targetAngle.getAsDouble();

    addRequirements(swerve);
  }

  public AdvancedSwerveDriveCommand(SwerveSubsystem swerve, double targetSpeed, double targetAngle, double radious, double turnByAngle, DoubleSupplier currentAngle, double startingAngle) {
    this.swerve = swerve;
    this.appliedSpeed = targetSpeed;
    this.targetAngle = targetAngle;
    this.radious = radious;
    this.turnByAngle = turnByAngle;
    this.currentAngle = currentAngle.getAsDouble();
    this.startingAngle = startingAngle;

    isCircle = true;

    double circumference = (Math.PI * radious) * 2;
    mDegrees = turnByAngle / circumference;

    addRequirements(swerve);
  }

  public AdvancedSwerveDriveCommand(SwerveSubsystem swerve) {
    isMotionTest = true;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isCircle) {
      appliedAngle = ((currentAngle + (mDegrees *  targetSpeed)) + targetAngle) / 2;
    } else {
      appliedSpeed = Constants.SwerveDrive.RoFtMBasedVolts(targetSpeedX + targetSpeedY) / Constants.SwerveDrive.driveMotorMaxVoltage;
      appliedAngle = (((90 * targetSpeedY) / (targetSpeedY + targetSpeedX)) + targetAngle) / 2;
    }

    if (isMotionTest) {
      if (swerve.getVelocity() >= Constants.SwerveDrive.testEndVelocity) {
        SmartDashboard.putNumber("SwerveDrive test ending velocity: ", swerve.getVelocity());
        SmartDashboard.putNumber("SwerveDrive test ending appliedSpeed: ", appliedSpeed);
        SmartDashboard.putNumber("SwerveDrive test ending front left appliedVoltage: ", swerve.getFrontLeftVoltage());
        SmartDashboard.putNumber("SwerveDrive test ending back left appliedVoltage: ", swerve.getBackLeftVoltage());

        isMotionTest = false;
        testsFinished = true;
      } else {
        appliedSpeed = appliedSpeed + Constants.SwerveDrive.testRampRate;
      }
    }

    swerve.setFrontLeft(appliedSpeed, appliedAngle);
    swerve.setFrontRight(appliedSpeed, appliedAngle);
    swerve.setBackLeft(appliedSpeed, appliedAngle);
    swerve.setBackRight(appliedSpeed, appliedAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isCircle) {
      if (currentAngle < startingAngle - Constants.SwerveDrive.circleDegreesTolerence || currentAngle > startingAngle + Constants.SwerveDrive.circleDegreesTolerence) {
        return true;
      } else {
        return false;
      }
    }

    if (testsFinished) {
      return true;
    }

    return false;
  }
}
