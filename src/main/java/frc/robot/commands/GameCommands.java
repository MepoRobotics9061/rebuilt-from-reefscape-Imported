package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.RobotFuel;
import frc.robot.subsystems.RobotFuelPivot;
import frc.robot.subsystems.RobotLauncher;
import frc.robot.subsystems.RobotClimber;
import frc.robot.subsystems.RobotCamera;
import frc.robot.commands.LimeLightCenterATagCommand;
import frc.robot.commands.LimeLightCoralPrepCommand;
import frc.robot.commands.LimeLightAlgaePrepCommand;
import frc.robot.commands.RotateUntilCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GameCommands /* extends SubsystemBase */{

  private final RobotFuel m_robotAlgae;

  private final RobotFuelPivot m_robotFuelPivot;

  private final RobotCamera m_robotCamera;

  private final RobotLauncher m_robotLauncher;

  private final RobotClimber m_robotClimber;

  private final Swerve s_Swerve;

  double gyro;

  double spinAmount;

  double tagX;

  double tagArea;

  double tagID;

  public GameCommands(
      RobotFuel robotAlgae,
      RobotFuelPivot robotFuelPivot,
      RobotCamera robotCamera,
      RobotLauncher robotLauncher,
      RobotClimber robotClimber,
      Swerve swerve) {
    m_robotAlgae = robotAlgae;

    m_robotFuelPivot = robotFuelPivot;

    m_robotCamera = robotCamera;

    m_robotLauncher = robotLauncher;

    m_robotClimber = robotClimber;

    s_Swerve = swerve;

  }

  public Command runFuelIntakeCommand(double speed) {
    return m_robotLauncher.intake(speed);
  }



  public Command runFuelLaunchCommand(double speed) {
    return m_robotLauncher.launch(speed);
  }

  // public Command runAlgaeIntakeCommand(double speed) {
  //   System.out.println("runAlgaeIntakeCommand: RUNNING");
  //   return m_robotAlgae.intake(speed);
  // }

  // public Command runAlgaeLaunchCommand(double speed) {
  //   System.out.println("runAlgaeLaunchCommand: RUNNING");
  //   return m_robotAlgae.launch(speed);
  // }

  public Command manualFuelPivotMove(double desiredValue) {
    return m_robotFuelPivot.manualPivotMove(desiredValue);
  }

  public Command fuelPivotPositionSetCommand(DoubleSupplier pivotPoint) {
    return m_robotFuelPivot.pivotPositionSet(pivotPoint);
  }

  // public Command pivotUpCommand(double speed) {
  // return m_robotCoralPivot.pivotUp(speed);
  // }

  // public Command pivotDownCommand(double speed) {
  // return m_robotCoralPivot.pivotDown(speed);
  // }

  public Command climberMoveCommand(double desiredValue) {
    return m_robotClimber.manualClimberMove(desiredValue);
  }

  public Command climberMoveCommandSet(double desiredValue) {
    return m_robotClimber.manualClimberMoveSet(desiredValue);
  }

  public Command driveCommand(DoubleSupplier vertical, DoubleSupplier horizontal, DoubleSupplier rotate) {
    return new TeleopSwerve(
        s_Swerve,
        () -> -vertical.getAsDouble(),
        () -> -horizontal.getAsDouble(),
        () -> -rotate.getAsDouble(),
        () -> false);
  }

  // public Command autoDriveCommand(double vertical, double horizontal, double rotate) {
  //   return this.runEnd(
  //     () -> {
  //     driveCommand(vertical, horizontal, rotate);
  //   }, () -> {
  //     driveCommand(0, 0, 0);
  //   });
  // }

  public Command driveRotateUntilCommand(DoubleSupplier angle) {
    return new RotateUntilCommand(s_Swerve, () -> angle.getAsDouble());
  }

  public Command centerATagCommand() {
    return new LimeLightCenterATagCommand(s_Swerve);
  }

  public Command coralPrepCommand() {
    return new LimeLightCoralPrepCommand(s_Swerve);
  }

  public Command algaePrepCommand() {
    return new LimeLightAlgaePrepCommand(s_Swerve);
  }
}