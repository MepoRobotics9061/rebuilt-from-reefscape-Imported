// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.autos.Autos;
import frc.robot.commands.LimeLightCenterATagCommand;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopSwerveLimit;
import frc.robot.subsystems.RobotIntake;
import frc.robot.subsystems.RobotCamera;
import frc.robot.subsystems.RobotLaunchChain;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.RobotIntakePivot;
import frc.robot.subsystems.RobotClimber;
import frc.robot.subsystems.RobotCamera;

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
  private final Autos m_autos;

  private final SendableChooser<Command> m_autoChooser;

  /* Variables */
  String climbMode = "Off (press twice)";

  Boolean isAuto = false;

  public RobotContainer() {

    m_autos = new Autos(
      m_robotClimber, m_robotIntake, m_robotIntakePivot, m_robotLaunchChain, m_robotCamera, s_Swerve
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

    configureButtonBindings();
    configureAutos();

    SmartDashboard.putString("Climb Mode", climbMode);
  }

  private void configureButtonBindings() {

    /*
      XBox:
        5 / LB = Intake Fuel
        7 / Back = Push Fuel Out of Intake
        6 / RB = Fire Fuel
        8 / Start = Push Fuel Out of Launch Chain
        4 / Y = Store Intake Mechanism
        1 / A = Move Intake Mechanism to a Usable Position
        POV [D-Pad] Up/Down = Move Climber
        9 / Left Stick = Toggle Climb Mode

      Joystick:

    */

    /* Driver Buttons */



    /* Operator Buttons */

    operator.button(Constants.Controller.LB).whileTrue(
      m_robotIntake.intake(.36)
    );

    operator.button(Constants.Controller.Back).whileTrue(
      m_robotIntake.launch(.1)
    );

    operator.povUp().whileTrue(
      m_robotClimber.manualClimberMove(-80)
    );

    operator.povDown().whileTrue(
      m_robotClimber.manualClimberMove(10)
    );

    /*
    The two m_robotLaunchChain commands need check to make sure the buttons are right.
    Start should slowly push the fuel backwards while RB should be the primary fire.
    */

    operator.button(Constants.Controller.RB).whileTrue(
      m_robotLaunchChain.indvSpeedCommand(1, 1)
    );
    
    operator.button(Constants.Controller.Start).whileTrue(
      m_robotLaunchChain.push(-.1)
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
      new InstantCommand( () -> {
        if(climbMode == "Off"){
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
    // m_autoChooser.addOption("2", m_autos.autoCommand2());
    // m_autoChooser.addOption("3", m_autos.autoCommand3());
    // m_autoChooser.addOption("TEST", m_autos.autoCommandTEST());

    m_autoChooser.setDefaultOption("2", m_autos.autoCommand1());

    SmartDashboard.putData("AutoCommand", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

}