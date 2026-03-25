package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.RobotCamera;
import frc.robot.subsystems.RobotClimber;
import frc.robot.subsystems.RobotIntake;
import frc.robot.subsystems.RobotIntakePivot;
import frc.robot.subsystems.RobotLaunchChain;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveUntilCommand;

public class Autos {

  private final RobotClimber m_robotClimber;

  private final RobotIntake m_robotIntake;

  private final RobotIntakePivot m_robotIntakePivot;

  private final RobotLaunchChain m_robotLaunchChain;

  private final RobotCamera m_robotCamera;

  private final Swerve s_Swerve;

  public Autos(
      RobotClimber robotClimber, RobotIntake robotIntake, RobotIntakePivot robotIntakePivot,
      RobotLaunchChain robotLaunchChain, RobotCamera robotCamera, Swerve swerve) {
    m_robotClimber = robotClimber;
    m_robotIntake = robotIntake;
    m_robotIntakePivot = robotIntakePivot;
    m_robotLaunchChain = robotLaunchChain;
    m_robotCamera = robotCamera;
    s_Swerve = swerve;
  }

  private Command driveCommand(double xSpeed, double ySpeed, double rSpeed) {
    return new DriveWithJoysticks(
        s_Swerve,
        () -> xSpeed,
        () -> ySpeed,
        () -> rSpeed,
        () -> true,
        () -> 1);
  }

  private Command DriveUntilCommand(double angle, double XPos, double YPos) {
    return new DriveUntilCommand(s_Swerve, () -> angle, XPos, YPos);
  }

  public Command autoCommandTime() {
    return Commands
        .sequence(
            driveCommand(-.2, 0, 0).withTimeout(1),
            m_robotLaunchChain.fire(1).withTimeout(2));
  }

}