package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotLaunchChain extends SubsystemBase {

  SparkMax leftWheel;
  SparkMax rightWheel;

  private DigitalInput limitSwitch;


  //Hopper Wheels
  public RobotLaunchChain() {
    final int leftWheelDeviceID = 13;
    final int rightWheelDeviceID = 14;
    leftWheel = new SparkMax(leftWheelDeviceID, MotorType.kBrushless);
    rightWheel = new SparkMax(rightWheelDeviceID, MotorType.kBrushless);

    limitSwitch = new DigitalInput(5);

  }

  public Command launch(double speed) {
    return this.runEnd(
        () -> {
            setWheelSpeed(speed);
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
            setWheelSpeed(-speed);
          // } else {
          //   stop();
          // }
        },
        () -> {
          stop();
        }
      );
  }

    public Command indvSpeedCommand(double speed1, double speed2) {
    return this.runEnd(
        () -> {
          // if(limitSwitch.get() == false) {
            setIndvWheelSpeed(SmartDashboard.getNumber("Bottom Hopper Speed", .3)*speed1, SmartDashboard.getNumber("Top Hopper Speed Speed", .3)*speed2);
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
    leftWheel.set(speed);
    rightWheel.set(-speed);
        System.out.println("runCoralLaunchCommand: RUNNING" + speed);
  }

    public void setIndvWheelSpeed(double speedL, double speedR) {
    leftWheel.set(speedL);
    rightWheel.set(-speedR);
  }

  public void stop() {
    leftWheel.set(0);
    rightWheel.set(0);
  }
}