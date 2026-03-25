package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotLaunchChain extends SubsystemBase {

  SparkMax fireWheel;
  SparkMax bottomWheel;
  SparkMax topWheel;

  /**
   * Creates a new RobotLaunchChain subsystem.
   */
  public RobotLaunchChain() {
    // Hopper Wheels
    final int bottomWheelDeviceID = 13;
    final int topWheelDeviceID = 14;
    bottomWheel = new SparkMax(bottomWheelDeviceID, MotorType.kBrushless);
    topWheel = new SparkMax(topWheelDeviceID, MotorType.kBrushless);
  }

  /**
   * Creates a command to push the hopper wheels at the specified speed.
   * 
   * @param speed the speed to push the hopper wheels at, in meters per second.
   * @return a command that pushes the hopper wheels at the specified speed.
   */
  public Command push(double speed) {
    return this.runEnd(
        () -> {
          setHopperSpeed(speed);
        },
        () -> {
          stop();
        });
  }

  /**
   * Creates a command to fire the hopper wheels at the specified speed.
   * 
   * @param speed the speed to fire the hopper wheels at, in meters per second.
   * @return a command that fires the hopper wheels at the specified speed.
   */
  public Command fire(double speed) {
    return this.runEnd(
        () -> {
          setHopperSpeed(-speed);
        },
        () -> {
          stop();
        });
  }

  /**
   * Creates a command to set the individual speeds of the bottom and top
   * hopper wheels.
   * 
   * @param speedB the speed multiplier for the bottom hopper wheel
   * @param speedT the speed multiplier for the top hopper wheel
   * @return a command that sets the individual speeds of the hopper wheels
   */
  public Command indvSpeedCommand(double speedB, double speedT) {
    return this.runEnd(
        () -> {
          setIndvWheelSpeed(SmartDashboard.getNumber("Bottom Hopper Speed", .45) * speedB,
              SmartDashboard.getNumber("Top Hopper Speed", .65) * speedT);
        },
        () -> {
          stop();
        });
  }

  /**
   * Sets the speed of both the bottom and top hopper wheels to the specified
   * speed.
   * The bottom wheel is set to the specified speed, while the top wheel is set to
   * the negative of the specified speed.
   * This is because the top and bottom hopper wheels are mounted on the same
   * shaft, so they must spin in opposite directions.
   * 
   * @param speed the speed to set the hopper wheels to, in meters per second.
   */
  public void setHopperSpeed(double speed) {
    bottomWheel.set(speed);
    topWheel.set(-speed);
  }

  /**
   * Sets the speed of the bottom and top hopper wheels to the specified speeds.
   * The bottom wheel is set to the specified speed, while the top wheel is set to
   * the negative of the specified speed.
   * This is because the top and bottom hopper wheels are mounted on the same
   * shaft, so they must spin in opposite directions.
   * 
   * @param speedB the speed to set the bottom hopper wheel to, in meters per
   *               second.
   * @param speedT the speed to set the top hopper wheel to, in meters per second.
   */
  public void setIndvWheelSpeed(double speedB, double speedT) {
    bottomWheel.set(speedB);
    topWheel.set(-speedT);

  }

  /**
   * Stops both the bottom and top hopper wheels.
   */
  public void stop() {
    bottomWheel.set(0);
    topWheel.set(0);
  }
}