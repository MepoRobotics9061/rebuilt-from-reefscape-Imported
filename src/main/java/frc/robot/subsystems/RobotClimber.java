package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotClimber extends SubsystemBase {

  SparkMax Climber;

  private SparkMaxConfig configWheel;

  private RelativeEncoder ClimberEncoder;

  private double GravityOffset = -0.06;

  public RobotClimber() {
    final int pivotWheelDeviceID = 11;
    Climber = new SparkMax(pivotWheelDeviceID, MotorType.kBrushless);
    configWheel = new SparkMaxConfig();
    configWheel.smartCurrentLimit(60);
    configWheel.idleMode(IdleMode.kBrake);
    Climber.configure(configWheel, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ClimberEncoder = Climber.getEncoder();

  }

  public Command manualClimberMove(double manualPosition) {
    return this.runEnd(
        () -> {
          voidClimberMove(manualPosition);
        },
        () -> {
          stop();
        });
  }

  public Command manualClimberMoveSet(double manualPosition) {
    return this.runEnd(
        () -> {
          System.out.println(": RUNNING");
          voidClimberMove(manualPosition);
        },
        () -> {
          doNothing();
        });
  }

  public void voidClimberMove(double manualPosition) {
    System.out.println("void climbermove: RUNNING");
    double targetSpeed = (manualPosition - ClimberEncoder.getPosition()) * .2 + GravityOffset;

    if (targetSpeed < -.3) {
      targetSpeed = -.3;
    }
    if (targetSpeed > .1) {
      targetSpeed = .1;
    }
    SmartDashboard.putNumber("Climb output", targetSpeed);
    setSpeed(targetSpeed);
  }

<<<<<<< HEAD
    public Command manualClimberMoveSet(double manualPosition) {
      return this.runEnd(
          () -> {
            System.out.println(": RUNNING");
            voidClimberMove(manualPosition);
          },
          () -> {
            doNothing();
          }
        );
    }
  
    public void voidClimberMove(double manualPosition) {
 System.out.println("void climbermove: RUNNING");
      double targetSpeed = (manualPosition - ClimberEncoder.getPosition()) * .2 + GravityOffset;
    System.out.println("runClimberMove: targetSpeed=" + manualPosition);
      if(targetSpeed < -.6) {
        targetSpeed = -.6;
      }
      if(targetSpeed > .3){
        targetSpeed = .3;
      }
 SmartDashboard.putNumber("Climb output", targetSpeed);
      setSpeed(targetSpeed);
    }
=======
  public Command ClimberMove(double speed) {
    return this.runEnd(
        () -> {
          System.out.println("runClimberMove: RUNNING" + speed);
          setSpeed(speed);
        },
        () -> {
          stop();
        });
  }

  public void setSpeed(double speed) {
    Climber.set(speed);
  }
>>>>>>> 26a03f6efb02d50ba0b573daed96b339fb62a432

  public void stop() {
    Climber.set(0);
  }

  public void doNothing() {

  }

  public void setPosition(double position) {
    // PositionForClimb = position;
  }

  // public boolean inPosition() {
  // return Math.abs(PositionForClimb - ClimberEncoderSmartDashboardValue) < .5;
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder", (ClimberEncoder.getPosition()));

  }
}