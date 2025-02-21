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

  double targetSpeedX, targetSpeedY, targetHeading, turnBySeconds, targetAngle, targetSpeed, radious, turnByAngle, currentAngle, mDegrees, startingAngle, circumference = 0.0;

  double appliedSpeed, appliedAngle = 0.0;

  boolean isCircle, isMotionTest, testsFinished = false;

  public AdvancedSwerveDriveCommand(SwerveSubsystem swerve, DoubleSupplier targetSpeedXIn, DoubleSupplier targetSpeedYIn, DoubleSupplier targetHeadingIn, DoubleSupplier turnBySecondsIn) {
    this.swerve = swerve;
    this.targetSpeedX = targetSpeedXIn.getAsDouble();
    this.targetSpeedY = targetSpeedYIn.getAsDouble();
    this.targetHeading = targetHeadingIn.getAsDouble();
    if (!(turnBySecondsIn == null)) {
      this.turnBySeconds = turnBySecondsIn.getAsDouble();
    } else {
      this.turnBySeconds = targetHeading / Constants.SwerveDrive.maxRotationalSpeed;
    }
    if (turnBySeconds == 0) {
      turnBySeconds = 1;
    }

    if (!(targetHeading == 0)) {
      targetAngle = (((90 * targetSpeedY) / (targetSpeedY + targetSpeedX)) + (((targetHeading / 360) * 45) / turnBySeconds)) / 2;
    } else {
      targetAngle = ((90 * targetSpeedY) / (targetSpeedY + targetSpeedX));
    }

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

    circumference = (Math.PI * radious) * 2;
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
      appliedAngle = ((currentAngle + (mDegrees *  targetSpeed)) + (((targetHeading / 360) * 45) / (circumference / targetSpeed))) / 2;

      swerve.setFrontLeft(appliedSpeed, ((currentAngle + (mDegrees *  targetSpeed)) + (((targetHeading / 360) * 45) / (circumference / targetSpeed))) / 2);
      swerve.setFrontRight(appliedSpeed, ((currentAngle + (mDegrees *  targetSpeed)) + (((targetHeading / 360) * 135) / (circumference / targetSpeed))) / 2);
      swerve.setBackLeft(appliedSpeed, ((currentAngle + (mDegrees *  targetSpeed)) + (((targetHeading / 360) * 45) / (circumference / targetSpeed))) / 2);
      swerve.setBackRight(appliedSpeed, ((currentAngle + (mDegrees *  targetSpeed)) + (((targetHeading / 360) * 135) / (circumference / targetSpeed))) / 2);

      
    } else {
      appliedSpeed = Constants.SwerveDrive.RoFtMBasedVolts(targetSpeedX + targetSpeedY) / Constants.SwerveDrive.driveMotorMaxVoltage;
      appliedAngle = ((90 * targetSpeedY) / (targetSpeedY + targetSpeedX));
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

    if (!(targetHeading == 0)) {
      swerve.setFrontLeft(appliedSpeed, ((((90 * targetSpeedY) / (targetSpeedY + targetSpeedX)) + (((targetHeading / 360) * 45) / turnBySeconds)) / 2));
      swerve.setFrontRight(appliedSpeed, ((((90 * targetSpeedY) / (targetSpeedY + targetSpeedX)) + (((targetHeading / 360) * 135) / turnBySeconds)) / 2));
      swerve.setBackLeft(appliedSpeed, ((((90 * targetSpeedY) / (targetSpeedY + targetSpeedX)) + (((targetHeading / 360) * 45) / turnBySeconds)) / 2));
      swerve.setBackRight(appliedSpeed, ((((90 * targetSpeedY) / (targetSpeedY + targetSpeedX)) + (((targetHeading / 360) * 135) / turnBySeconds)) / 2));
    } else {
      swerve.setFrontLeft(appliedSpeed, appliedAngle);
      swerve.setFrontRight(appliedSpeed, appliedAngle);
      swerve.setBackLeft(appliedSpeed, appliedAngle);
      swerve.setBackRight(appliedSpeed, appliedAngle);
    }

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
