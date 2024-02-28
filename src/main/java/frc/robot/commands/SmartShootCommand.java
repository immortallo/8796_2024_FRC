package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LauncherSubsystem;

public class SmartShootCommand extends SequentialCommandGroup {
  /**
   * Creates a new SmartShootCommand.
   * @param launcherSubsystem
   * @param shootSpeed The speed at which to shoot the balls
   * POSITIVE values will shoot the balls!
   */
  public SmartShootCommand(LauncherSubsystem launcherSubsystem, double shootSpeed) {
    super(
        new RunCommand(() -> launcherSubsystem.setDownSpeed(-shootSpeed)).withTimeout(2),
        new RunCommand(() -> launcherSubsystem.setBothSpeed(-shootSpeed)));
  }
}
