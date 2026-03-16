package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class LimeLightAlgaePrepCommand extends Command {
  private Swerve s_Swerve;

  public LimeLightAlgaePrepCommand(
  Swerve s_Swerve){
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {
    s_Swerve.algaePrepVoid();
  }

}