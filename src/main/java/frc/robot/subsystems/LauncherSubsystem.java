package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
  private final WPI_VictorSPX launcherMotorUp;
  private final CANSparkMax launcherMotorDown;

  private final VictorSPX launcherMotorUpFollower;
  private final CANSparkMax launcherMotorDownFollower;

  public LauncherSubsystem() {
    // Motors on the left side are leaders
    launcherMotorUp = new WPI_VictorSPX(LauncherConstants.kLauncherMotorLeftUpCanID);
    launcherMotorDown = new CANSparkMax(LauncherConstants.kLauncherMotorLeftDownCanID, MotorType.kBrushed);

    // Motors on the right side are followers
    launcherMotorUpFollower = new VictorSPX(LauncherConstants.kLauncherMotorRightUpCanID);
    launcherMotorDownFollower = new CANSparkMax(LauncherConstants.kLauncherMotorRightDownCanID, MotorType.kBrushed);

    launcherMotorUpFollower.follow(launcherMotorUp);
    launcherMotorDownFollower.follow(launcherMotorDown);
  }

  public void setBothSpeed(double speed) {
    launcherMotorUp.set(speed);
    launcherMotorDown.set(speed);
  }

  public void setSpeed(double speedUp, double speedDown) {
    launcherMotorUp.set(speedUp);
    launcherMotorDown.set(speedDown);
  }

  public void stop() {
    launcherMotorUp.stopMotor();
    launcherMotorDown.stopMotor();
  }

  public void setUpSpeed(double speed) {
    launcherMotorUp.set(speed);
  }

  public void setDownSpeed(double speed) {
    launcherMotorDown.set(speed);
  }
}
