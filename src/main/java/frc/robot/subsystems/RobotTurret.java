package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.andymark.jni.AM_CAN_HexBoreEncoder;
import com.andymark.jni.AM_CAN_HexBoreEncoder.AM_EncoderStatus;
import com.andymark.jni.AM_CAN_HexBoreEncoder.AM_Encoder_Telemetry;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RobotTurret extends SubsystemBase {

    private final SparkMax turretMotor;
    private final SparkMax backMotor;
    private final SparkMax fireMotor;
    private final RelativeEncoder turretEncoder;
    private double TurretEncoderValue;

    private final AM_CAN_HexBoreEncoder hexEncoder;



    public RobotTurret() {
        turretMotor = new SparkMax(17, MotorType.kBrushless);
        backMotor = new SparkMax(15, MotorType.kBrushless);
        
        //NEEDS MOVED OVER TO LAUNCH CHAIN
        fireMotor = new SparkMax(10, MotorType.kBrushless);

        //The device's default CAN is 0. Change it using AndyMark CAN interface utility
hexEncoder = new AM_CAN_HexBoreEncoder(20);
//Reset Report Period to the default of 10ms
hexEncoder.resetReportPeriod();

//All data the HexBore Encoder provides can be retrieved by using these two lines
AM_Encoder_Telemetry telemetryData = hexEncoder.getTelemetry();
AM_EncoderStatus statusData = hexEncoder.getStatus();

        // Optional: load configuration
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kBrake);
        config.smartCurrentLimit(30);

        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turretEncoder = turretMotor.getEncoder();
        turretEncoder.setPosition(0); // Zero at startup
    }

    public Command setTurretSpeedCommand(double RSpeed, double ASpeed, double FSpeed) {

        return this.runEnd(
            () -> {
                setTurretRSpeed(RSpeed);
                setTurretAngleSpeed(ASpeed);
                setFireSpeed(FSpeed);
            }, () -> {
                stopTurret();
                stopFire();
                stopAngle();
            }
        );

    }

        public Command testingSpeed(double speed) {
      return this.runEnd(
        () -> {
            System.out.println("runTurretTest: RUNNING" + speed);
          setTurretAngleSpeed(speed);
        }, () -> {
          stopAngle();
        }
      );
    }

     public Command testingTurretSpeed(double speed) {
      return this.runEnd(
        () -> {
            System.out.println("runTurretTest: RUNNING" + speed);
          setTurretRSpeed(speed);
        }, () -> {
          stopAngle();
        }
      );
    }


       public Command setFireSpeedCommand(double FSpeed) {

        return this.runEnd(
            () -> {
        System.out.println("runFireTest: RUNNING" + FSpeed);
                setFireSpeed(FSpeed);
            }, () -> {
             
                stopFire();
              
            }
        );

    }

    // Basic open-loop control
    public void setTurretRSpeed(double speed) {
        turretMotor.set(speed);
    }

    public void setTurretAngleSpeed(double speed) {
        backMotor.set(speed);
    }

    public void setFireSpeed(double speed) {
        fireMotor.set(speed);
    }

    // Stop turret
    public void stopTurret() {
        turretMotor.stopMotor();
    }

     public void stopFire() {
        fireMotor.stopMotor();
    }

    public void stopAngle() {
        backMotor.stopMotor();
    }

    // Read encoder position (rotations)
    public double getPosition() {
        return turretEncoder.getPosition();
    }

    // Read encoder velocity (RPM)
    public double getVelocity() {
        return turretEncoder.getVelocity();
    }

    @Override public void periodic() {

        AM_Encoder_Telemetry telemetryData = hexEncoder.getTelemetry();
AM_EncoderStatus statusData = hexEncoder.getStatus();
double degrees = hexEncoder.getAngleDegrees();
 double degreesPerSec = hexEncoder.getVelocityDegPerSec();
SmartDashboard.putNumber("backshooterangleDegrees", degrees);
SmartDashboard.putNumber("backshooterVelocity (DegPerSec)", degreesPerSec);

//System.out.println("hex position Test" + degrees);

        TurretEncoderValue = turretEncoder.getPosition();
        SmartDashboard.putNumber("TurretAnglePostition", TurretEncoderValue);


    }
}