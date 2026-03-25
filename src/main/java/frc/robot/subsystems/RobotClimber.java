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

  /**
   * Creates a new RobotClimber subsystem.
   */
  public RobotClimber() {
    final int pivotWheelDeviceID = 11;
    Climber = new SparkMax(pivotWheelDeviceID, MotorType.kBrushless);
    configWheel = new SparkMaxConfig();
    configWheel.smartCurrentLimit(60);
    configWheel.idleMode(IdleMode.kBrake);
    Climber.configure(configWheel, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ClimberEncoder = Climber.getEncoder();

  }

  /**
   * Creates a command to move the climber to the specified position.
   * 
   * @param manualPosition the position to move the climber to, in meters.
   * @return a command that moves the climber to the specified position.
   */
  public Command manualClimberMove(double manualPosition) {
    return this.runEnd(
        () -> {
          voidClimberMove(manualPosition);
        },
        () -> {
          stop();
        });
  }

  /**
   * Creates a command to move the climber to the specified position.
   * 
   * @param manualPosition the position to move the climber to, in meters.
   * @return a command that moves the climber to the specified position.
   */
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

  /**
   * Moves the climber to the specified position.
   * 
   * @param manualPosition the position to move the climber to, in meters.
   */
  public void voidClimberMove(double manualPosition) {
    System.out.println("void climbermove: RUNNING");
    double targetSpeed = (manualPosition - ClimberEncoder.getPosition()) * .2 + GravityOffset;
    System.out.println("runClimberMove: targetSpeed=" + manualPosition);
    if (targetSpeed < -.6) {
      targetSpeed = -.6;
    }
    if (targetSpeed > .3) {
      targetSpeed = .3;
    }
    SmartDashboard.putNumber("Climb output", targetSpeed);
    setSpeed(targetSpeed);
  }

  /**
   * Creates a command to move the climber at the specified speed.
   * 
   * @param speed the speed to move the climber at, in meters per second.
   * @return a command that moves the climber at the specified speed.
   */
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

  /**
   * Sets the speed of the climber to the specified speed.
   * 
   * @param speed the speed to set the climber to, in meters per second.
   */
  public void setSpeed(double speed) {
    Climber.set(speed);
  }

  /**
   * Stops the climber.
   */
  public void stop() {
    Climber.set(0);
  }

  /**
   * Does nothing.
   */
  public void doNothing() {

  }

  /**
   * Sets the position of the climber to the specified position.
   * 
   * @param position the position to set the climber to, in meters.
   */
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