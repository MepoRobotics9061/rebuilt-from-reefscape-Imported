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

  private double ClimberEncoderSmartDashboardValue;

  private double GravityOffset = -0.06;

  private double PositionForClimb = -5;

  public RobotClimber() {
    final int pivotWheelDeviceID = 11;
    Climber = new SparkMax(pivotWheelDeviceID, MotorType.kBrushless);
    ClimberEncoder = Climber.getEncoder();
    }

    public Command manualClimberMove(double manualPosition) {
      return this.runEnd(
          () -> {
            voidClimberMove(manualPosition);
          },
          () -> {
            stop();
          }
        );
    }

    public Command manualClimberMoveSet(double manualPosition) {
      return this.runEnd(
          () -> {
            voidClimberMove(manualPosition);
          },
          () -> {
            doNothing();
          }
        );
    }
  
    public void voidClimberMove(double manualPosition) {
      
      double targetSpeed = (manualPosition - ClimberEncoderSmartDashboardValue) * .2 + GravityOffset;

      if(targetSpeed < -.5) {
        targetSpeed = -.5;
      }
      if(targetSpeed > .2) {
        targetSpeed = .2;
      }
      setSpeed(targetSpeed);
    }

    // public Command ClimberMove(double speed) {
    //   return this.runEnd(
    //       () -> {
    //          System.out.println("runClimberMove: RUNNING" + speed);
    //   setSpeed(speed);
    //       }, 
    //       () -> {
    //         stop();
    //       }
    //   );
    // }


    public void setSpeed(double speed) {
      Climber.set(speed);
    }
  
    public void stop() {
      Climber.set(0);
    }

    public void doNothing(){

    }

    public void setPosition(double position) {
      PositionForClimb = position;
    }

    public boolean inPosition() {
      return Math.abs(PositionForClimb - ClimberEncoderSmartDashboardValue) < .5;
    }

  @Override public void periodic() {
    ClimberEncoderSmartDashboardValue = ClimberEncoder.getPosition();
    SmartDashboard.putNumber("Elevator Encoder", ClimberEncoderSmartDashboardValue);
 
    
  }
}