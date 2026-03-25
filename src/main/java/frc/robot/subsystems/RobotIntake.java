package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotIntake extends SubsystemBase {

  SparkMax wheel;

  private SparkMaxConfig configWheel;

  /**
   * Creates a new RobotIntake subsystem.
   */
  public RobotIntake() {
    final int wheelDeviceID = 12;
    wheel = new SparkMax(wheelDeviceID, MotorType.kBrushless);
    configWheel = new SparkMaxConfig();
    configWheel.smartCurrentLimit(40);
    configWheel.idleMode(IdleMode.kBrake);
    wheel.configure(configWheel, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Creates a command to run the intake wheels backwards at the specified speed.
   * 
   * @param speed the speed to run the intake wheels backwards at, in meters per
   *              second.
   * @return a command that runs the intake wheels backwards at the specified
   *         speed.
   */
  public Command backwards(double speed) {
    return this.runEnd(
        () -> {
          setWheelSpeed(-speed);
        },
        () -> {
          stop();
        });
  }

  /**
   * Creates a command to run the intake wheels forwards at the specified speed.
   * 
   * @param speed the speed to run the intake wheels forwards at, in meters per
   *              second.
   * @return a command that runs the intake wheels forwards at the specified
   *         speed.
   */
  public Command forwards(double speed) {
    return this.runEnd(
        () -> {
          setWheelSpeed(speed);
        },
        () -> {
          stop();
        });
  }

  /**
   * Sets the speed of the intake wheels to the specified speed.
   * 
   * @param speed the speed to set the intake wheels to, in meters per second.
   */
  public void setWheelSpeed(double speed) {
    wheel.set(speed);
  }

  /**
   * Stops the intake wheels.
   */
  public void stop() {
    wheel.set(0);
  }

}