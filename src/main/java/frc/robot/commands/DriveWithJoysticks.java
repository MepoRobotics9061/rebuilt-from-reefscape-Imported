
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
//import frc.robot.subsystems.PoseEstimator;

import frc.robot.subsystems.Swerve;

public class DriveWithJoysticks extends Command {

  private final Swerve swerveSubsystem;
 // private final PoseEstimator poseEstimator;

  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier rotation;
  private final BooleanSupplier relative;
  private final DoubleSupplier maxSpeed;

  private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

  //public DriveWithJoysticks(Swerve swerveSubsystem, PoseEstimator poseEstimator, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier relative, DoubleSupplier maxSpeed) {
   
    public DriveWithJoysticks(Swerve swerveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier relative, DoubleSupplier maxSpeed) {
    this.swerveSubsystem = swerveSubsystem;
   // this.poseEstimator = poseEstimator;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
    this.relative = relative;
    this.maxSpeed = maxSpeed;
    this.xLimiter = new SlewRateLimiter(100.0);
    this.yLimiter = new SlewRateLimiter(100.0);
    this.turnLimiter = new SlewRateLimiter(100.0);

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SmartDashboard.putNumber("Joy Y", translationX.getAsDouble());
    SmartDashboard.putNumber("Joy X", translationY.getAsDouble());
    SmartDashboard.putNumber("Joy Rot", rotation.getAsDouble());


    // if(relative.getAsBoolean()){
    //   swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
    //   modifyAxis(translationX.getAsDouble(), maxSpeed.getAsDouble(), yLimiter) * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND,
    //   modifyAxis(translationY.getAsDouble(), maxSpeed.getAsDouble(), xLimiter) * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND,
    //   modifyAxis(rotation.getAsDouble(), maxSpeed.getAsDouble(), turnLimiter) * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
    //   poseEstimator.getPoseRotation()
    // ));
    // } else {
    //   swerveSubsystem.drive2(new ChassisSpeeds(
    //   modifyAxis(translationX.getAsDouble(), maxSpeed.getAsDouble(), yLimiter) * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND,
    //   modifyAxis(translationY.getAsDouble(), maxSpeed.getAsDouble(), xLimiter) * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND,
    //   modifyAxis(rotation.getAsDouble(), maxSpeed.getAsDouble(), turnLimiter) * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    // }

      //      swerveSubsystem.drive2(new ChassisSpeeds(
      // modifyAxis(translationX.getAsDouble(), maxSpeed.getAsDouble(), yLimiter) * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND,
      //  modifyAxis(translationY.getAsDouble(), maxSpeed.getAsDouble(), xLimiter) * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND,
      //  modifyAxis(rotation.getAsDouble(), maxSpeed.getAsDouble(), turnLimiter) * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
  
             swerveSubsystem.drive2(ChassisSpeeds.fromFieldRelativeSpeeds(
      modifyAxis(translationX.getAsDouble(), maxSpeed.getAsDouble(), yLimiter) * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND,
       modifyAxis(translationY.getAsDouble(), maxSpeed.getAsDouble(), xLimiter) * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND,
       modifyAxis(rotation.getAsDouble(), maxSpeed.getAsDouble(), turnLimiter) * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
       ,swerveSubsystem.getYawField()));
  
  
      }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Applies various modifications to the input (also squares the joystick input)
   * @param value the input of the joystick
   * @param speedModifyer how much to slow the joystick down by
   * @param limiter how much to slow the inputs down by
   * @return the modified joystick values
   */
  private double modifyAxis(double value, double speedModifyer, SlewRateLimiter limiter){
    value = MathUtil.applyDeadband(value, 0.02);
    value = Math.copySign(value * value, value);
    value = value*speedModifyer;
    value = limiter.calculate(value);
    if(Math.abs(value)*Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND <= Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND*0.01){
      value = 0.0;
    }
    return value;
  }
}
