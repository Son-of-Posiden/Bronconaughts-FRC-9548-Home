// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Shooter {
    public static final int leftMotorID = 13;
    public static final int rightMotorID = 14;
  }

  public static class Arm {
    public static final int armMotorID = 15;

    public static final double kp = 0.02;
    public static final double ki = 0.0;
    public static final double kd = 0.0;

    public static final double encoderToAngleCoefficent = 0.0;

    public static final double maxAngle = 1.0;
    public static final double minAngle = 0.0;
  }

  public static class LinearSlide {
    public static final int slideMotorID = 16;
    
    public static final double kp = 0.02;
    public static final double ki = 0.0;
    public static final double kd = 0.0;

    public static final double encoderToMetersCoefficent = 0.0;

    public static final double slideHightFromFloor = 0.2;

    public static final double maxHight = 1.0;
    public static final double minHight = 0.0;
  }

  public static class Lift {
    public static final int liftMotorID = 17;

    public static final double maxHight = 1.0;
    public static final double minHight = 0.0;
  }

  public static class SwerveDrive {
    public static final double MAX_SPEED = 2.0; //YGASL uses 4.4196
    public static final double maxRotationalSpeed = 360; //Degrees Per Second

    public static final double driveMotorMaxVoltage = 12;
    public static final double turnMotorMaxVoltage = 12;

    public static final double MOI = 1.803;

    public static final double circleDegreesTolerence = 3; //Can not be zero.

    public static final double interationsPerSecond = 20;

    public static final double testEndVelocity = 0.5;
    public static final double testRampRate = 0.1;

    public static final double metersPerSecondWhileTesting = 2.0;
    public static final double voltageWhileTestingMotion = 8.0;

    public static final double degreesPerSecondWhileTesting = 120.0;
    public static final double voltageWhileTestingAngularRotation = 6.0;

    public static double RoFtMBasedVolts(double targetMetersPerSecond) {
      return Math.pow(voltageWhileTestingMotion, (targetMetersPerSecond / metersPerSecondWhileTesting)); 
    } //Ratio of Force to Motion. Something I made up for my math.
    public static double RoFtABasedVolts(double targetDegreesPerSecond) {
      return Math.pow(voltageWhileTestingAngularRotation, (targetDegreesPerSecond / degreesPerSecondWhileTesting)); 
    } //Ratio of Force to Angular motion. Something I made up for my math.
  }

  public static class Vision {
    public static final double turnKp = 0.01;
    public static final double distanceKp = 0.4; //0.8 for carpet maybe. Kp to convert area to meters.

    public static final double shooterShotKp = 0.05;
    public static final double maxBallVelocity = 2.0; //m/s of the ball.

    public static final double distanceWhileTestingShot = 5.0; //Meters, how far away you were.
    public static final double voltageWhileTestingShot = 4.0; //Voltage, to hit where you want to.

    public static final double maxMotorVoltage = 12;

    public static double RoFtDBasedVolts(double currentDistance) {
      return Math.pow(voltageWhileTestingShot, (currentDistance / distanceWhileTestingShot)); 
    } //Ratio of Force to Distance. Something I made up for my math.

    public static final double targetHightRampSpeed = 0.1; //In meters per second. Higher will make the shot less accurate, but shooter sooner if there is delay.
  }

  public static class PhysicsConstants {
    public static final double gravitationalConstant = 9.81;
  }
}
