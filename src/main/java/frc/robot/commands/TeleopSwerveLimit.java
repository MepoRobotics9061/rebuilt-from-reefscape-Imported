package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerveLimit extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private DoubleSupplier speedSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(4.5);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(4.5);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(4.5);

  public TeleopSwerveLimit(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      DoubleSupplier speedSup,
      BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.speedSup = speedSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/

    double speedMult = ((speedSup.getAsDouble()+1.0) / 3.0)+ 0.33;

    double translationVal =
        translationLimiter.calculate(
          MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));

 

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed * speedMult),
        rotationVal * Constants.Swerve.maxAngularVelocity * speedMult,
        robotCentricSup.getAsBoolean(),
        true);
  }

  // @Override
  // public Command zeroGyroCommand(){
  //   return this.run(() -> {gyro.zeroYaw();});
  // }
}