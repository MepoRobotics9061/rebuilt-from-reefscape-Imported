package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;

public class DriveUntilCommand extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier angle;
  private double XPos;
  private double YPos;

  public DriveUntilCommand(
      Swerve s_Swerve, DoubleSupplier angle, double XPos, double YPos) {
    this.s_Swerve = s_Swerve;
    this.angle = angle;
    this.XPos = XPos;
    this.YPos = YPos;
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {
    s_Swerve.driveUntilVoid(angle.getAsDouble(), XPos, YPos);
  }

}