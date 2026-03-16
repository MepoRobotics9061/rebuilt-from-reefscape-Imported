package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotFuel extends SubsystemBase {

  SparkMax wheel;

  private SparkMaxConfig configWheel;

  private DigitalInput limitSwitch;

  public RobotFuel() {
    final int wheelDeviceID = 12;
    wheel = new SparkMax(wheelDeviceID, MotorType.kBrushless);
    configWheel = new SparkMaxConfig();
    configWheel.smartCurrentLimit(40);
    configWheel.idleMode(IdleMode.kBrake);
    wheel.configure(configWheel, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    limitSwitch = new DigitalInput(6);
  }

  public Command launch(double speed) {
    return this.runEnd(
        () -> {
          setWheelSpeed(-speed);
        },
        () -> {
          stop();
        }
    );
  }

  public Command intake(double speed) {
    return this.runEnd(
        () -> {
          // if(limitSwitch.get() == false) {
            setWheelSpeed(speed);
          // } else {
          //   stop();
          // }
        },
        () -> {
          stop();
        }
    );
  }

  public void setWheelSpeed(double speed) {
    wheel.set(speed);
  }

  public void stop() {
    wheel.set(0);
  }

}