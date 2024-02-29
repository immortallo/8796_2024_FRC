// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax leftMotor;
  private final CANSparkMax leftMotorFollower;
  private final CANSparkMax rightMotor;
  private final CANSparkMax rightMotorFollower;

  private final DifferentialDrive differentialDrive;

  public DriveSubsystem() {
    leftMotor = new CANSparkMax(DriveConstants.kLeftMotorPort, MotorType.kBrushed);
    leftMotorFollower = new CANSparkMax(DriveConstants.kLeftMotorFollowerPort, MotorType.kBrushed);
    rightMotor = new CANSparkMax(DriveConstants.kRightMotorPort, MotorType.kBrushed);
    rightMotorFollower = new CANSparkMax(DriveConstants.kRightMotorFollowerPort, MotorType.kBrushed);

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
