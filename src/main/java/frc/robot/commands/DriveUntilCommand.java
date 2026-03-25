package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;

public class DriveUntilCommand extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier angle;
  private double XPos;
  private double YPos;

  /**
   * Creates a new DriveUntilCommand command.
   * 
   * @param s_Swerve The swerve subsystem to use.
   * @param angle    The angle supplier.
   * @param XPos     The X position.
   * @param YPos     The Y position.
   */
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