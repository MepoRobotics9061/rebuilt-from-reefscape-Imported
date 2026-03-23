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

  public RobotLaunchChain() {
    // Hopper Wheels
    final int bottomWheelDeviceID = 13;
    final int topWheelDeviceID = 14;
    bottomWheel = new SparkMax(bottomWheelDeviceID, MotorType.kBrushless);
    topWheel = new SparkMax(topWheelDeviceID, MotorType.kBrushless);
  }

  public Command push(double speed) {
    return this.runEnd(
        () -> {
          setHopperSpeed(speed);
        },
        () -> {
          stop();
        });
  }

  public Command fire(double speed) {
    return this.runEnd(
        () -> {
          setHopperSpeed(-speed);
        },
        () -> {
          stop();
        });
  }

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

  public void setHopperSpeed(double speed) {
    bottomWheel.set(speed);
    topWheel.set(-speed);
  }

  public void setIndvWheelSpeed(double speedB, double speedT) {
    bottomWheel.set(speedB);
    topWheel.set(-speedT);
  }

  public void stop() {
    bottomWheel.set(0);
    topWheel.set(0);
  }
}