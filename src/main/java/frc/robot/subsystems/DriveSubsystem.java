// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_VictorSPX leftMotor;
  private final VictorSPX leftMotorFollower;
  private final WPI_VictorSPX rightMotor;
  private final VictorSPX rightMotorFollower;

  private final DifferentialDrive differentialDrive;

  public DriveSubsystem() {
    leftMotor = new WPI_VictorSPX(DriveConstants.kLeftMotorPort);
    leftMotorFollower = new VictorSPX(DriveConstants.kLeftMotorFollowerPort);
    rightMotor = new WPI_VictorSPX(DriveConstants.kRightMotorPort);
    rightMotorFollower = new VictorSPX(DriveConstants.kRightMotorFollowerPort);

    // Right motor should go forward when given a positive value.
    // so we set it to be inverted.
    rightMotor.setInverted(true);
    // Follow the leaders
    // from now on, the leftMotor and rightMotor are the leaders
    // and you can control them as if they were a single motor
    leftMotorFollower.follow(leftMotor);
    rightMotorFollower.follow(rightMotor);

    // create a differential drive using the leader motors
    // this will use both motors on each side as a single motor
    // thanks to the followers
    differentialDrive = new DifferentialDrive(leftMotor, rightMotor);
  }

  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }

  public void arcadeDrive(double fwd, double rot, boolean squareInputs) {
    differentialDrive.arcadeDrive(fwd, rot, squareInputs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
