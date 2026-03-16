package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;

public class RotateUntilCommand extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier angle;

  public RotateUntilCommand(
  Swerve s_Swerve, DoubleSupplier angle
  ){
    this.s_Swerve = s_Swerve;
    this.angle = angle;
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {
    s_Swerve.rotateUntilVoid(angle.getAsDouble());
  }

}