package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import frc.lib.math.Conversions;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.SwerveModuleConstants;

import frc.robot.Constants;

public class SwerveModule {

  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private TalonFX angleMotor;
  private SparkFlex driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkClosedLoopController driveController;
  // private final SparkClosedLoopController angleController;

  private SparkFlexConfig configDrive;
  private SparkFlexConfig configAngle;

  private PositionVoltage angleSetter = new PositionVoltage(0);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    // angleEncoder = new DutyCycleEncoder(moduleConstants.cancoderID);
    // configAngleEncoder();

    /* Angle Motor Config */
    // angleMotor = new SparkMax(moduleConstants.angleMotorID,
    // MotorType.kBrushless);

    angleMotor = new TalonFX(moduleConstants.angleMotorID, "rio");

    // var angleConfiguration = new TalonFXConfiguration();
    configAngleMotorTalon(moduleConstants.cancoderID);

    // integratedAngleEncoder = angleMotor.getEncoder();
    // angleController = angleMotor.getClosedLoopController();
    // configAngle = new SparkMaxConfig();
    // configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new SparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    configDrive = new SparkFlexConfig();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous
    // controller which
    // REV and CTRE are not

    // desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    if (moduleNumber == 1) {
      SmartDashboard.putString("Mod 1 cur angle", getState().angle.toString());
      SmartDashboard.putString("Mod 1 desired angle", desiredState.angle.toString());
    }

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void resetToAbsolute() {
    // double absolutePosition = getCanCoder().getDegrees() -
    // angleOffset.getDegrees();
    // integratedAngleEncoder.setPosition(absolutePosition);

    double test = (getCanCoder().getDegrees() - angleOffset.getDegrees());

    double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(),
        Constants.Swerve.angleGearRatio);
    // angleMotor.setSelectedSensorPosition(absolutePosition);
    Timer.delay(0.5);
    angleMotor.setPosition(test / 360);

    System.out.println("get cancoder " + getCanCoder().getDegrees());
    System.out.println("get offset " + angleOffset.getDegrees());
    System.out.println("reset to absolute " + test);

  }

  private void configAngleEncoder() {
    // angleEncoder.configFactoryDefault();
    // CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    // angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    // angleEncoder.setDistancePerRotation(360.0);
  }

  /* Configure settings for the steering motor */

  private void configAngleMotorTalon(int cancoderID) {
    angleMotor.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0.kV = 0.0;
    talonfxConfigs.Slot0.kS = 0.0;
    talonfxConfigs.Slot0.kP = Constants.Swerve.angleKP;
    talonfxConfigs.Slot0.kI = Constants.Swerve.angleKI;
    talonfxConfigs.Slot0.kD = Constants.Swerve.angleKD;
    talonfxConfigs.ClosedLoopGeneral.ContinuousWrap = true;
    talonfxConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;
    talonfxConfigs.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
    // talonfxConfigs.CurrentLimits.SupplyCurrentThreshold =
    // Constants.Swerve.anglePeakCurrentLimit;
    // talonfxConfigs.CurrentLimits.SupplyTimeThreshold =
    // Constants.Swerve.anglePeakCurrentDuration;
    talonfxConfigs.Voltage.PeakForwardVoltage = 10;
    talonfxConfigs.Voltage.PeakReverseVoltage = -10;
    talonfxConfigs.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;

    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // talonfxConfigs.MotorOutput.Inverted = ModuleConstants.angleMotorInvert;
    angleMotor.getConfigurator().apply(talonfxConfigs);

  }

  private void configAngleMotor() {
    // angleMotor.restoreFactoryDefaults();
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    configAngle.smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    configAngle.inverted(true);
    // configAngle.idleMode(Constants.Swerve.angleNeutralMode);
    configAngle.idleMode(IdleMode.kCoast);
    configAngle.encoder.positionConversionFactor(Constants.Swerve.angleConversionFactor);
    configAngle.closedLoop.pid(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);
    // angleController.setI(Constants.Swerve.angleKI);
    // angleController.setD(Constants.Swerve.angleKD);
    configAngle.closedLoop.velocityFF(Constants.Swerve.angleKFF);
    // angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    // angleMotor.burnFlash();
    resetToAbsolute();
    // angleMotor.configure(configAngle, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    angleMotor.getConfigurator().apply(slot0Configs);

  }

  private void configDriveMotor() {
    // driveMotor.restoreFactoryDefaults();
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    // driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    // driveMotor.setInverted(Constants.Swerve.driveInvert);
    // driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    // driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    // driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    // driveController.setP(Constants.Swerve.angleKP);
    // driveController.setI(Constants.Swerve.angleKI);
    // driveController.setD(Constants.Swerve.angleKD);
    // driveController.setFF(Constants.Swerve.angleKFF);
    // driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    // driveMotor.burnFlash();
    // driveEncoder.setPosition(0.0);

    configDrive.smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    configDrive.inverted(true);
    configDrive.idleMode(IdleMode.kCoast);
    configDrive.encoder.velocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    configDrive.encoder.positionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    configDrive.closedLoop.pid(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);
    configDrive.closedLoop.velocityFF(Constants.Swerve.angleKFF);
    resetToAbsolute();
    driveMotor.configure(configDrive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond, ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    // angleController.setReference(angle.getDegrees(), ControlType.kPosition);

    double angleToSet = angle.getDegrees() / 360;

    if (moduleNumber == 1) {
      SmartDashboard.putNumber("Mod 1 desiredspeed", desiredState.speedMetersPerSecond);
      SmartDashboard.putNumber("Mod 1 setangle", angle.getDegrees());
      SmartDashboard.putNumber("Mod 1 setangleTOSET", angleToSet);
    }

    angleMotor.setControl(angleSetter.withPosition(angleToSet));

    // create a position closed-loop request, voltage output, slot 0 configs
    // final PositionVoltage m_request = new
    // PositionVoltage(Conversions.degreesToFalcon(angle.getDegrees(),
    // Constants.Swerve.angleGearRatio)).withSlot(0);

    // set position to 10 rotations
    // angleMotor.setControl(m_request.withPosition(10));

    lastAngle = angle;
  }

  /**
   * Stops the module from driving and turning when called
   */
  public void stop() {
    driveMotor.stopMotor();
    angleMotor.stopMotor();
  }

  public double getAngleD() {
    return angleMotor.getPosition().getValueAsDouble();
  }

  public double getPosition() {
    return angleMotor.getPosition().getValueAsDouble();
  }

  private Rotation2d getAngle() {
    // return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    // var test = angleMotor.getPosition().refresh().getValue();
    // return Rotation2d.fromRotations(test.abs(null));

    // Get the raw position in rotations from Phoenix 6 API
    double rotations = angleMotor.getPosition().getValueAsDouble();

    // Convert to radians and wrap into Rotation2d
    return Rotation2d.fromRotations(rotations);
  }

  public Rotation2d getCanCoder() {
    var test = angleEncoder.getAbsolutePosition();
    var test2 = test.getValueAsDouble();
    // return Rotation2d.fromDegrees(angleEncoder.get() * 360);
    return Rotation2d.fromDegrees(test2 * 360);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }
}