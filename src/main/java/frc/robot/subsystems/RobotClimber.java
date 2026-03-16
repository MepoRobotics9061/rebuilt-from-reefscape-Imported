package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class RobotClimber extends SubsystemBase {

  SparkMax Climber;

  private RelativeEncoder ClimberEncoder;

  private double ClimberEncoderValue;

  private double gainSpeed = -0.06;

  private double targetPosition = -5;

  private double angleOffset = 0;

  private boolean isAuto = false;

  public RobotClimber() {
    final int pivotWheelDeviceID = 11;
    Climber = new SparkMax(pivotWheelDeviceID, MotorType.kBrushless);
    ClimberEncoder = Climber.getEncoder();
    }

    public Command manualClimberMove(double manualAngle) {
      return this.runEnd(
          () -> {
            voidClimberMove(manualAngle);
          },
          () -> {
            stop();
          }
        );
    }

    public Command manualClimberMoveSet(double manualAngle) {
      return this.runEnd(
          () -> {
            voidClimberMove(manualAngle);
          },
          () -> {
            doNothing();
          }
        );
    }
  
    public void voidClimberMove(double manualAngle) {
      manualAngle = Math.max(Math.min(manualAngle - angleOffset, -4), -95);
      
      double targetSpeed = (manualAngle - ClimberEncoderValue) * .2 + gainSpeed;

      if(targetSpeed < -.5) {
        targetSpeed = -.5;
      }
      if(targetSpeed > .2) {
        targetSpeed = .2;
      }
      setSpeed(targetSpeed);
    }

    public Command ClimberMove(double speed) {
      return this.runEnd(
          () -> {
             System.out.println("runClimberMove: RUNNING" + speed);
      setSpeed(speed);
          }, 
          () -> {
            stop();
          }
      );
    }


    public void setSpeed(double speed) {
      Climber.set(speed);
    }
  
    public void stop() {
      Climber.set(0);
    }

    public void doNothing(){

    }

    public void setPosition(double position) {
      targetPosition = position;
    }

    public boolean inPosition() {
      return Math.abs(targetPosition - ClimberEncoderValue) < .5;
    }

  @Override public void periodic() {
    ClimberEncoderValue = ClimberEncoder.getPosition();
    SmartDashboard.putNumber("Elevator Encoder", ClimberEncoderValue);

 
    
  }
}