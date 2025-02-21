// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public double x, y, area;

  public double deadbandDistance = 1.1;

  public VisionSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    SmartDashboard.putNumber("area ", area);
  }

  public Command turnTowardAprilTag(SwerveSubsystem swerve) {
    return run(()-> {
        swerve.setHeadingCorrection(false);

        swerve.setFrontLeft((x * Constants.Vision.turnKp), -45.0);
        swerve.setFrontRight(-(x * Constants.Vision.turnKp), 45.0);
        swerve.setBackLeft((x * Constants.Vision.turnKp), 45.0);
        swerve.setBackRight(-(x * Constants.Vision.turnKp), -45.0);
      }
    );
  }

  public Command moveTowardAprilTag(SwerveSubsystem swerve) {
    return run(()-> {
        swerve.setHeadingCorrection(false);

        swerve.setFrontLeft((deadbandDistance - area) * Constants.Vision.distanceKp, 0.0);
        swerve.setFrontRight((deadbandDistance - area) * Constants.Vision.distanceKp, 0.0);
        swerve.setBackLeft((deadbandDistance - area) * Constants.Vision.distanceKp, 0.0);
        swerve.setBackRight((deadbandDistance - area) * Constants.Vision.distanceKp, 0.0);
      }
    );
  }

  public Command moveAndAlignTowardAprilTag(SwerveSubsystem swerve) {
    return run(()-> {
        swerve.setHeadingCorrection(false);

        swerve.setFrontLeft((deadbandDistance - area) * Constants.Vision.distanceKp + (x * Constants.Vision.turnKp), 0.0);
        swerve.setFrontRight((deadbandDistance - area) * Constants.Vision.distanceKp + (-(x * Constants.Vision.turnKp)), 0.0);
        swerve.setBackLeft((deadbandDistance - area) * Constants.Vision.distanceKp + (x * Constants.Vision.turnKp), 0.0);
        swerve.setBackRight((deadbandDistance - area) * Constants.Vision.distanceKp + (-(x * Constants.Vision.turnKp)), 0.0);
      }
    );
  }

  public double getTx() {
    return x;
  }

  public double calculatePerfectShot(double targetHight, double range, double vi, ArmSubsystem arm, LinearSlideSubsystem slide) {
    double theta = (Math.pow(Math.sin((range * Constants.PhysicsConstants.gravitationalConstant) / Math.pow(vi, 2)), -1)) / 2;

    double speed = vi / Constants.Vision.maxBallVelocity;

    arm.setSetpoint(theta);

    return speed;
  }

  public double calculateShot(double targetHight, double targetDistance, ArmSubsystem arm, LinearSlideSubsystem slide) {
    double realTargetHight = targetHight - Constants.LinearSlide.slideHightFromFloor;
    double t = 2 * (realTargetHight/ Constants.PhysicsConstants.gravitationalConstant);
    double vx = 2 * (realTargetHight/ t);
    double vi = vx / Math.sin(arm.getCurrentAngle());
    double vy = vi*Math.sin(arm.getCurrentAngle());

    double bigT = (vy - Math.sqrt(Math.pow(vy, 2) - ((2 * Constants.PhysicsConstants.gravitationalConstant) * realTargetHight))) / Constants.PhysicsConstants.gravitationalConstant;
    double distance = vx * bigT;

    while (distance < targetDistance) {
      realTargetHight = realTargetHight + Constants.Vision.targetHightRampSpeed;

      t = 2 * (realTargetHight/ Constants.PhysicsConstants.gravitationalConstant);
      vx = 2 * (realTargetHight/ t);
      vi = vx / Math.sin(arm.getCurrentAngle());
      vy = vi*Math.sin(arm.getCurrentAngle());
      bigT = (vy - Math.sqrt(Math.pow(vy, 2) - ((2 * Constants.PhysicsConstants.gravitationalConstant) * realTargetHight))) / Constants.PhysicsConstants.gravitationalConstant;
      distance = vx * bigT;
    }

    double speed = vi / Constants.Vision.maxBallVelocity;

    return speed;
  }

  public double calculateShotByVolts() {
    if (Constants.Vision.RoFtDBasedVolts(area * Constants.Vision.distanceKp) <= 12) {
      return Constants.Vision.RoFtDBasedVolts(area * Constants.Vision.distanceKp) / Constants.Vision.maxMotorVoltage;
    } 
    else {
      return 1;
    }
  }
}
