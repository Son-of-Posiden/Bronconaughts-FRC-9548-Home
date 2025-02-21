// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

/** Add your docs here. */
public class mathExtras {
  public static double deadband(double value, double lowNumber) {
      if (Math.abs(value) < lowNumber) {
        value = 0.0;
      }
      return value;
  }

  public static double codeStop(double value, double lowNumber, double highNumber) {
    if (value < lowNumber) {
      value = lowNumber;
    }
    if (value > highNumber) {
      value = highNumber;
    }
    return value;
  }

  public static double continuousAngleShortestDistance(double currentAngle, double targetAngle) {
    double angle = (Math.abs(currentAngle) + Math.abs(targetAngle)) * Math.signum(currentAngle) * Math.signum(targetAngle);
    return angle;
  }
}
