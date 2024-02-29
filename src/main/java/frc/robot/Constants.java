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

    // TODO: if the Y axis is inverted, change this to true,
    // If going forward makes the robot go backwards, set this to true
    public static final boolean kYAxisInverted = true;

    // TODO: if the X axis is inverted, change this to true,
    // If turning left makes the robot turn right, set this to true
    public static final boolean kXAxisInverted = false;
  }

  public static class DriveConstants {
    public static final int kLeftMotorPort = 1;
    public static final int kLeftMotorFollowerPort = 2;
    public static final int kRightMotorPort = 3;
    public static final int kRightMotorFollowerPort = 4;
  }

  public static class LauncherConstants {
    public static final int kLauncherMotorLeftUpPWM = 1;     // CTRE VictorSPX
    public static final int kLauncherMotorRightUpPWM = 0;    // CTRE VictorSPX
    public static final int kLauncherMotorLeftDownPWM = 2;   // REV SparkMax
    public static final int kLauncherMotorRightDownPWM = 3;  // REV SparkMax

    public static final double kWaitTimeSeconds = 2; // The time to wait for the launcher to spin up
  }

  public static class AutoConstants {
    public static final double kShootSpeed = 1;
    public static final double kDriveSpeed = 0.4;
    public static final double kShootTime = 4; // including the wait time to spin up (which is 2 seconds)
    public static final double kDriveTime = 2;
  }
}
