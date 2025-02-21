// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  double maximumSpeed = Units.feetToMeters(4.5);
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive swerveDrive;

  SwerveModule[] modules;

  RobotConfig robotConfig;

  public SwerveSubsystem() {
    try
    {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.SwerveDrive.MAX_SPEED,
                                                                  new Pose2d(new Translation2d(Meter.of(0), //Was 1
                                                                                               Meter.of(0)), //Was 4
                                                                             Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    modules = swerveDrive.getModules();
    
    for (int count = 0; count < modules.length; count++) {
      SmartDashboard.putNumber("Module Number " + count + " absolute pos", modules[count].getAbsolutePosition());
      SmartDashboard.putNumber("Module Number " + count + " raw absolute pos", modules[count].getRawAbsolutePosition());
    }

    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.0020645, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.0020645, 0.0, 0.0) // Rotation PID constants
            ),
            robotConfig, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }
   
  public void setHeadingCorrection(boolean doesCorrect) {
    swerveDrive.setHeadingCorrection(doesCorrect);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    return run(() -> {
      setHeadingCorrection(true);

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    
    return run(() -> {
      setHeadingCorrection(true);
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        false,
                        false);
    });
  }

  public double getGyroYaw() {
    return swerveDrive.getYaw().getDegrees();
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public void resetPose(Pose2d pose2d) {
    swerveDrive.resetOdometry(pose2d);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    swerveDrive.drive(speeds);
  }

  public double getVelocity() {
    return swerveDrive.getRobotVelocity().vxMetersPerSecond + swerveDrive.getRobotVelocity().vyMetersPerSecond;
  }

  public double getCurrentAngle() {
    return swerveDrive.getOdometryHeading().getDegrees();
  }

  public double getFrontLeftVoltage() {
    return modules[0].getDriveMotor().getVoltage();
  }
  public double getFrontRightVoltage() {
    return modules[1].getDriveMotor().getVoltage();
  }
  public double getBackLeftVoltage() {
    return modules[2].getDriveMotor().getVoltage();
  }
  public double getBackRightVoltage() {
    return modules[3].getDriveMotor().getVoltage();
  }
  
  public void setFrontLeft(double speed, double angle) {
    modules[0].getDriveMotor().set(speed);
    modules[0].setAngle(angle);
  }
  public void setFrontRight(double speed, double angle) {
    modules[1].getDriveMotor().set(speed);
    modules[1].setAngle(angle);
  }
  public void setBackLeft(double speed, double angle) {
    modules[2].getDriveMotor().set(speed);
    modules[2].setAngle(angle);
  }
  public void setBackRight(double speed, double angle) {
    modules[3].getDriveMotor().set(speed);
    modules[3].setAngle(angle);
  }
}
