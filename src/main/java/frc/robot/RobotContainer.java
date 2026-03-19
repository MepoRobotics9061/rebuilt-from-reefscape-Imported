// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.autos.Autos;
import frc.robot.commands.LimeLightCenterATagCommand;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.GameCommands;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopSwerveLimit;
import frc.robot.subsystems.RobotIntake;
import frc.robot.subsystems.RobotCamera;
import frc.robot.subsystems.RobotLaunchChain;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.RobotIntakePivot;
import frc.robot.subsystems.RobotClimber;
import frc.robot.subsystems.RobotCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final CommandJoystick driver = new CommandJoystick(0);

  private final CommandXboxController operator = new CommandXboxController(1);

  /* Subsystems */
  private final RobotIntake m_robotIntake = new RobotIntake();

  private final RobotIntakePivot m_robotIntakePivot = new RobotIntakePivot();

  private final RobotLaunchChain m_robotLaunchChain = new RobotLaunchChain();

  private final RobotClimber m_robotClimber = new RobotClimber();

  private final RobotCamera m_robotCamera = new RobotCamera();

  private final Swerve s_Swerve = new Swerve(m_robotCamera); 
  
  /* Commands */
  
  private final GameCommands m_gameCommands;
  private final Autos m_autos;
  private final SendableChooser<Command> m_autoChooser;

  String climbMode = "Off (press twice)";

  Boolean isAuto = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_gameCommands = new GameCommands(
      m_robotIntake,
      m_robotIntakePivot,
      m_robotCamera,
      m_robotLaunchChain,
      m_robotClimber,
      s_Swerve
    );

    m_autos = new Autos(
      m_gameCommands, m_robotClimber, m_robotIntakePivot
    );

    m_autoChooser = new SendableChooser<Command>();

      s_Swerve.setDefaultCommand(new DriveWithJoysticks(
      s_Swerve,
    //poseEstimator,
      () -> -driver.getY(),
      () -> -driver.getX(),
      () -> driver.getZ(),
      () -> true,
      () -> 1.0));

    // Configure the button bindings
    configureButtonBindings();
    configureAutos();

    SmartDashboard.putString("Climb Mode", climbMode);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /* Driver Buttons */



    /*  XBox:
          5, 6 [Bumpers] = Intake
          Trigger = Fire
          1, 2, 3, 4 [A, B, X, Y] = (Coral)/(Algae) Pivots
          9 [Left Stick]= Toggle Pivots
          POV [D-Pad] = (Elevator)/(Half Elevator)
          10 [Right Stick] = Toggle Elevator
          7, 8 [Select and Start]= Lights

        Joystick:
          7 = Center A Tag
          9 = Reef A Tag
          11 = Processor A Tag
        */

      /* Operator Buttons */

      operator.button(Constants.Controller.X).whileTrue(
        m_robotIntake.intake(.36)
      );

      operator.button(Constants.Controller.B).whileTrue(
        m_robotIntake.launch(.1)
      );

      operator.button(Constants.Controller.LB).whileTrue(
        m_robotLaunchChain.indvSpeedCommand(-1, -1)
      );

      operator.button(Constants.Controller.RB).whileTrue(
        m_robotLaunchChain.indvSpeedCommand(1, 1)
      );

      operator.button(Constants.Controller.Y).whileTrue(
        m_robotIntakePivot.manualPivotMove(6.8)
      );

      operator.button(Constants.Controller.A).whileTrue(
        m_robotIntakePivot.manualPivotMove(0)
      );

      operator.povDown().and(() -> climbMode == "On").whileTrue(
        m_robotClimber.ClimberMove(-.1)
      );

      operator.povUp().and(() -> climbMode == "On").whileTrue(
        m_robotClimber.ClimberMove(.1)
      );

      operator.button(Constants.Controller.LeftStick).onTrue(
        new InstantCommand( () -> {if(climbMode == "Off"){
          climbMode = "On";
        }else{
          climbMode = "Off";
        }
          SmartDashboard.putString("Climb Mode", climbMode);
      })
      );
  }

  private void configureAutos() {
    m_autoChooser.addOption("1", m_autos.autoCommand1());
    m_autoChooser.addOption("2", m_autos.autoCommand2());
    m_autoChooser.addOption("3", m_autos.autoCommand3());
    m_autoChooser.addOption("TEST", m_autos.autoCommandTEST());

    m_autoChooser.setDefaultOption("2", m_autos.autoCommand2());

    SmartDashboard.putData("AutoCommand", m_autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

}