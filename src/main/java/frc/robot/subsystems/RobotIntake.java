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

  public RobotIntake() {
    final int wheelDeviceID = 12;
    wheel = new SparkMax(wheelDeviceID, MotorType.kBrushless);
    configWheel = new SparkMaxConfig();
    configWheel.smartCurrentLimit(40);
    configWheel.idleMode(IdleMode.kBrake);
    wheel.configure(configWheel, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command backwards(double speed) {
    return this.runEnd(
        () -> {
          setWheelSpeed(-speed);
        },
        () -> {
          stop();
        });
  }

  public Command forwards(double speed) {
    return this.runEnd(
        () -> {
          setWheelSpeed(speed);
        },
        () -> {
          stop();
        });
  }

  public void setWheelSpeed(double speed) {
    wheel.set(speed);
  }

  public void stop() {
    wheel.set(0);
  }

}