package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
  private final PWMVictorSPX launcherMotorUp;
  private final PWMVictorSPX launcherMotorDown;

  private final PWMVictorSPX launcherMotorUpFollower;
  private final PWMVictorSPX launcherMotorDownFollower;

  public LauncherSubsystem() {
    // Motors on the left side are leaders
    launcherMotorUp = new PWMVictorSPX(LauncherConstants.kLauncherMotorLeftUpPWM);
    launcherMotorDown = new PWMVictorSPX(LauncherConstants.kLauncherMotorLeftDownPWM);

    // Motors on the right side are followers
    launcherMotorUpFollower = new PWMVictorSPX(LauncherConstants.kLauncherMotorRightUpPWM);
    launcherMotorDownFollower = new PWMVictorSPX(LauncherConstants.kLauncherMotorRightDownPWM);

    // Invert the followers, because they are on the right side
    launcherMotorUpFollower.setInverted(true);
    launcherMotorDownFollower.setInverted(true);

    launcherMotorDown.addFollower(launcherMotorDownFollower);
    launcherMotorUp.addFollower(launcherMotorUpFollower);
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
