
package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  // private final Pigeon2 gyro;
  private final AHRS gyro;

  private SwerveDriveOdometry swerveOdometry;

  private SwerveModule[] m_SwerveMods;

  private RobotCamera m_robotCamera;

  private Field2d field;

  private double gyroValue;
  private Rotation2d gyroRot2d;
  private double tagDistance;
  private double rotateErrorAmount;
  private double XPosErrorAmount;
  private double YPosErrorAmount;

  private Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private Pose2d m_pose;

  SwerveDriveOdometry m_odometry;

  private double gyroATagSpinAmount;

  public Swerve(RobotCamera robotCamera) {

    gyro = new AHRS(NavXComType.kMXP_SPI);

    gyro.zeroYaw();

    m_SwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)

    };

    field = new Field2d();

    SmartDashboard.putData("Field", field);

    m_robotCamera = robotCamera;

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    // Creating my odometry object from the kinematics object and the initial wheel
    // positions.
    // Here, our starting pose is 5 meters along the long end of the field and in
    // the
    // center of the field along the short end, facing the opposing alliance wall.
  
  
    // m_odometry = new SwerveDriveOdometry(
    //     m_kinematics, gyro.getRotation2d(),
    //     new SwerveModulePosition[] {
    //         new SwerveModulePosition(m_SwerveMods[0].getPosition(),
    //             Rotation2d.fromDegrees(m_SwerveMods[0].getAngleD())),
    //         new SwerveModulePosition(m_SwerveMods[1].getPosition(),
    //             Rotation2d.fromDegrees(m_SwerveMods[1].getAngleD())),
    //         new SwerveModulePosition(m_SwerveMods[2].getPosition(),
    //             Rotation2d.fromDegrees(m_SwerveMods[2].getAngleD())),
    //         new SwerveModulePosition(m_SwerveMods[3].getPosition(),
    //             Rotation2d.fromDegrees(m_SwerveMods[3].getAngleD())),
    //     }, new Pose2d(2.0, 7, new Rotation2d()));



    // swerveOdometry = new
    // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics,
    // getYaw(), new Pose2d(2.0, 7, new Rotation2d()));
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYawField())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : m_SwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Main controlling method for driving swerve based on desired speed of
   * drivetrian
   * 
   * @param chassisSpeeds Desired speed of drivetrain
   */
  public void drive2(ChassisSpeeds chassisSpeeds) {

    SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    setModuleStates2(states);
  }

  /**
   * Set the desired state of all the modules
   * 
   * @param states Desired module states
   */
  public void setModuleStates2(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);
    m_SwerveMods[0].setDesiredState(states[0], true);
    m_SwerveMods[1].setDesiredState(states[1], true);
    m_SwerveMods[2].setDesiredState(states[2], true);
    m_SwerveMods[3].setDesiredState(states[3], true);
  }

  /**
   * Stops all movement for swerve modules
   */
  public void stop() {
    m_SwerveMods[0].stop();
    m_SwerveMods[1].stop();
    m_SwerveMods[2].stop();
    m_SwerveMods[3].stop();
  }

  public void centerATagVoid() {
    drive(new Translation2d(0, m_robotCamera.tagX / 40), gyroATagSpinAmount, false, true);
  }

  public void coralPrepVoid() {
    if (m_robotCamera.tagArea != 0) {
      // drive(new Translation2d(-(10 - m_robotCamera.tagArea) / 20,
      // m_robotCamera.tagX / 40), gyroATagSpinAmount / 2 ,
      var xSp = 0.0;
      xSp = (15.5 - tagDistance) / 120;
      if (xSp > 0.12) {
        xSp = 0.12;
      }
      if (xSp < -0.12) {
        xSp = -0.12;
      }
      System.out.println(xSp);

      var sideSp = 0.0;
      sideSp = m_robotCamera.tagX / 120;
      if (sideSp > 0.1) {
        sideSp = 0.1;
      }
      if (sideSp < -0.1) {
        sideSp = -0.1;
      }
      System.out.println("side");
      System.out.println(sideSp);

      drive(new Translation2d(xSp, sideSp), gyroATagSpinAmount / 2,
          false, true);
    }
  }

  public void algaePrepVoid() {
    if (m_robotCamera.tagArea != 0) {
      // drive(new Translation2d(-(14 - m_robotCamera.tagArea) / 20,
      // m_robotCamera.tagX / 40), gyroATagSpinAmount / 2,
      var xSp = 0.0;
      xSp = (15.5 - tagDistance) / 120;
      if (xSp > 0.1) {
        xSp = 0.1;
      }
      if (xSp < -0.1) {
        xSp = -0.1;
      }

      drive(new Translation2d(xSp, m_robotCamera.tagX / 120), gyroATagSpinAmount / 2,
          false, true);
    }
  }

  public void driveUntilVoid(double desiredXPos, double desiredYPos, double desiredAngle) {

    rotateErrorAmount = gyroValue - desiredAngle;

    XPosErrorAmount = field.getRobotPose().getX() - desiredXPos;
    YPosErrorAmount = field.getRobotPose().getY() - desiredYPos;

    if (rotateErrorAmount > 180) {
      rotateErrorAmount -= 360;
    }

    if (rotateErrorAmount < -180) {
      rotateErrorAmount += 360;
    }

    rotateErrorAmount = rotateErrorAmount / 15;

    if ((.05 < rotateErrorAmount) && (rotateErrorAmount < .2)) {
      rotateErrorAmount = .2;
    }

    else if ((-.2 < rotateErrorAmount) && (rotateErrorAmount < -0.05)) {
      rotateErrorAmount = -.2;
    }

    if (rotateErrorAmount > 4) {
      rotateErrorAmount = 4;
    }

    if (rotateErrorAmount < -4) {
      rotateErrorAmount = -4;
    }

    if ((.05 < XPosErrorAmount) && (XPosErrorAmount < .1)) {
      XPosErrorAmount = .1;
    }

    else if ((-.1 < XPosErrorAmount) && (XPosErrorAmount < -0.05)) {
      XPosErrorAmount = -.1;
    }

    if (XPosErrorAmount > 5) {
      XPosErrorAmount = .5;
    }

    if (XPosErrorAmount < -.5) {
      XPosErrorAmount = -.5;
    }

    if ((.05 < YPosErrorAmount) && (YPosErrorAmount < .1)) {
      YPosErrorAmount = .1;
    }

    else if ((-.1 < YPosErrorAmount) && (YPosErrorAmount < -0.05)) {
      YPosErrorAmount = -.1;
    }

    if (YPosErrorAmount > 5) {
      YPosErrorAmount = .5;
    }

    if (YPosErrorAmount < -.5) {
      YPosErrorAmount = -.5;
    }

    drive(new Translation2d(XPosErrorAmount, YPosErrorAmount), -rotateErrorAmount, false, true);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : m_SwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  // public Pose2d getPose() {
  //   // return swerveOdometry.getPoseMeters();
  //   return m_pose;
  // }

  // public void resetOdometry(Pose2d pose) {
  //   // swerveOdometry.resetPosition(pose, getYaw());
  // }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : m_SwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public Command zeroGyro() {
    return this.run(() -> {
      gyro.zeroYaw();
    });
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public Rotation2d getYawField() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw() + 180)
        : Rotation2d.fromDegrees(gyro.getYaw() + 180);
  }

  // public double rotateUntilAmount(double desiredAngle) {
  // return 180 - desiredAngle;
  // }

  @Override
  public void periodic() {
    // // swerveOdometry.update(getYaw(), getStates());
    // field.setRobotPose(getPose());

    for (SwerveModule mod : m_SwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " getAngle", mod.getAngleD());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }

    // gyroValue = gyro.getYaw() + 180;
    gyroValue = gyro.getYaw();
    gyroRot2d = gyro.getRotation2d();

    // m_pose = m_odometry.update(gyroRot2d,
    //     new SwerveModulePosition[] {
    //         new SwerveModulePosition(m_SwerveMods[0].getPosition(),
    //             Rotation2d.fromDegrees(m_SwerveMods[0].getAngleD())),
    //         new SwerveModulePosition(m_SwerveMods[1].getPosition(),
    //             Rotation2d.fromDegrees(m_SwerveMods[1].getAngleD())),
    //         new SwerveModulePosition(m_SwerveMods[2].getPosition(),
    //             Rotation2d.fromDegrees(m_SwerveMods[2].getAngleD())),
    //         new SwerveModulePosition(m_SwerveMods[3].getPosition(),
    //             Rotation2d.fromDegrees(m_SwerveMods[3].getAngleD())) });

    // field.setRobotPose(getPose());

    // SmartDashboard.putNumber("X Position", field.getRobotPose().getX());
    // SmartDashboard.putNumber("Y Position", field.getRobotPose().getY());
    // SmartDashboard.putNumber("R Position", field.getRobotPose().getRotation().getDegrees());

    SmartDashboard.putNumber(
        "Gyro", gyroValue);

    double errorAmount = m_robotCamera.spinAmount - gyroValue;

    tagDistance = 14 / Math.tan((23 + m_robotCamera.tagY) / 57.29577) / 3;

    SmartDashboard.putNumber("Tag Distance", tagDistance);

    if (errorAmount > 180) {
      errorAmount -= 360;
    }

    else if (errorAmount < -180) {
      errorAmount += 360;
    }

    gyroATagSpinAmount = (errorAmount) / 30;

    if ((.05 < gyroATagSpinAmount) && (gyroATagSpinAmount < .2)) {
      gyroATagSpinAmount = .2;
    }

    else if ((-.2 < gyroATagSpinAmount) && (gyroATagSpinAmount < -0.05)) {
      gyroATagSpinAmount = -.2;
    }

    if (gyroATagSpinAmount > 2) {
      gyroATagSpinAmount = 2;
    }

    if (gyroATagSpinAmount < -2) {
      gyroATagSpinAmount = -2;
    }

  }
}