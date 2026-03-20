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
import java.util.function.DoubleSupplier;

public class RobotIntakePivot extends SubsystemBase {

  SparkMax pivotWheel;

  private SparkMaxConfig configWheel;

  private RelativeEncoder pivotEncoder;

  private double pivotEncoderValue;

  private double targetPosition;

   private double gravitySpeed = -0.04;
   private double gain = .14;
   private double maxPosSpeed = .08;
   private double maxNegSpeed = -.13;

  public RobotIntakePivot() {
    final int pivotWheelDeviceID = 16;
    pivotWheel = new SparkMax(pivotWheelDeviceID, MotorType.kBrushless);
    configWheel = new SparkMaxConfig();
    configWheel.smartCurrentLimit(40);
     configWheel.idleMode(IdleMode.kCoast);
    pivotWheel.configure(configWheel, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotEncoder = pivotWheel.getEncoder();
    }

    public Command pivotPositionSet(DoubleSupplier fuelPivotPoint) {
      return this.runEnd(() -> {
          SmartDashboard.putNumber("Fuel Pivot Point", fuelPivotPoint.getAsDouble());
        }, () -> {
          stop();
        }
      );
    }

    public Command manualPivotMove(double targetAngle) {
      return this.runEnd(() -> {
  
        SmartDashboard.putNumber("Fuel Pivot Point", targetAngle);
      
        double targetSpeed = (targetAngle - pivotEncoderValue) * gain + gravitySpeed;

        if(targetSpeed < maxNegSpeed) {
          targetSpeed = maxNegSpeed;
        }
        if(targetSpeed > maxPosSpeed) {
          targetSpeed = maxPosSpeed;
        }

        if (targetAngle > 6 && pivotEncoderValue > 6) {
          setSpeed(.03);
        } else {
          setSpeed(targetSpeed);
        }
        
        SmartDashboard.putNumber("Fuel Pivot SetSpeed", targetSpeed);
          }, () -> {
            stop();
          }
        );
    }

    public void voidPivotMove(double targetAngle) {

      SmartDashboard.putNumber("Fuel Pivot Point", targetAngle);

      double targetSpeed = (targetAngle - pivotEncoderValue) * gain + gravitySpeed;

      if(targetSpeed < maxNegSpeed) {
        targetSpeed = maxNegSpeed;
      }
      if(targetSpeed > maxPosSpeed) {
        targetSpeed = maxPosSpeed;
      }
      setSpeed(targetSpeed);
      SmartDashboard.putNumber("Fuel Pivot SetSpeed", targetSpeed);

    }

    
    public Command testingSpeed(double speed) {
      return this.runEnd(() -> {
          setSpeed(speed);
        }, () -> {
          stop();
        }
      );
    }
  
    public void setSpeed(double speed) {
      pivotWheel.set(speed);
    }
  
    public void stop() {
      pivotWheel.set(0);
    }

  @Override public void periodic() {
    pivotEncoderValue = pivotEncoder.getPosition();
    SmartDashboard.putNumber("Fuel Pivot Encoder", pivotEncoderValue);
    targetPosition = SmartDashboard.getNumber("Fuel Pivot Point", 0);
  }
}