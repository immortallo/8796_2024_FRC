// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SmartShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();

  private final Joystick controller = new Joystick(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    setDefaultCommands();
  }

  private void setDefaultCommands() {
    driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> driveSubsystem.arcadeDrive(
                OperatorConstants.kYAxisInverted ? -1 : 1 * controller.getY(),
                OperatorConstants.kXAxisInverted ? -1 : 1 * controller.getX()),
            driveSubsystem));

    launcherSubsystem.setDefaultCommand(
        new RunCommand(launcherSubsystem::stop, launcherSubsystem));
  }

  private void configureBindings() {
    new JoystickButton(controller, 1).whileTrue(
        new SmartShootCommand(launcherSubsystem, 1));

    new JoystickButton(controller, 2).whileTrue(
        new SmartShootCommand(launcherSubsystem, -0.4));

    new JoystickButton(controller, 4).whileTrue(
        new SmartShootCommand(launcherSubsystem, 0.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This will shoot the balls for 4 seconds and then drive backwards for 2 seconds.
    return new SmartShootCommand(launcherSubsystem, AutoConstants.kShootSpeed).withTimeout(AutoConstants.kShootTime)
        .andThen(new RunCommand(() -> driveSubsystem.arcadeDrive(-AutoConstants.kDriveSpeed, 0, false)).withTimeout(AutoConstants.kDriveTime));
  }
}
