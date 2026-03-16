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
import frc.robot.autos.Autos;
import frc.robot.commands.LimeLightCenterATagCommand;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.GameCommands;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopSwerveLimit;
import frc.robot.subsystems.RobotFuel;
import frc.robot.subsystems.RobotCamera;
import frc.robot.subsystems.RobotLauncher;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.RobotFuelPivot;
import frc.robot.subsystems.RobotClimber;
import frc.robot.subsystems.RobotTurret;
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
  private final RobotFuel m_robotFuel = new RobotFuel();

  private final RobotFuelPivot m_robotFuelPivot = new RobotFuelPivot();

  private final RobotLauncher m_robotLauncher = new RobotLauncher();

  private final RobotClimber m_robotClimber = new RobotClimber();

  //private final RobotTurret m_robotTurret = new RobotTurret();

  private final RobotCamera m_robotCamera = new RobotCamera();

  private final Swerve s_Swerve = new Swerve(m_robotCamera); 
  
  /* Commands */
  
  private final GameCommands m_gameCommands;
  private final Autos m_autos;
  private final SendableChooser<Command> m_autoChooser;

  String controllerMode = SmartDashboard.getString("Controller Mode", "Coral");

  Boolean isAuto = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_gameCommands = new GameCommands(
      m_robotFuel,
      m_robotFuelPivot,
      m_robotCamera,
      m_robotLauncher,
      m_robotClimber,
      s_Swerve
    );

    m_autos = new Autos(
      m_gameCommands, m_robotClimber, m_robotFuelPivot
    );

    m_autoChooser = new SendableChooser<Command>();

    // s_Swerve.setDefaultCommand(
    
    //   new TeleopSwerve(
    //     s_Swerve,
    //     () -> driver.getRawAxis(1) * driver.getRawAxis(1) * driver.getRawAxis(1),
    //     () -> driver.getRawAxis(0) * driver.getRawAxis(0) * driver.getRawAxis(0),
    //     () -> driver.getRawAxis(2) * driver.getRawAxis(2) * driver.getRawAxis(2) * .5,
    //     () -> true));



    // s_Swerve.setDefaultCommand(
    
    // new TeleopSwerveLimit(
    //   s_Swerve,
    //   () -> driver.getRawAxis(1) * driver.getRawAxis(1) * driver.getRawAxis(1),
    //   () -> driver.getRawAxis(0) * driver.getRawAxis(0) * driver.getRawAxis(0),
    //   () -> driver.getRawAxis(2) * driver.getRawAxis(2) * driver.getRawAxis(2) * .5,
    //   () -> -driver.getRawAxis(3) ,
    //   () -> true));


          s_Swerve.setDefaultCommand(new DriveWithJoysticks(
      s_Swerve,
    //  poseEstimator,
      () -> -driver.getY(),
      () -> -driver.getX(),
      () -> -driver.getZ(),
      () -> true,
      () -> 1.0));



    // m_robotFuelPivot.setDefaultCommand(
    //   new RunCommand(
    //     () -> m_robotFuelPivot.voidPivotMove(SmartDashboard.getNumber("Fuel Pivot Point", 0)), 
    //     m_robotFuelPivot
    //   )
    // );

    // m_robotClimber.setDefaultCommand(
    //   new RunCommand(
    //     () -> m_robotClimber.voidClimberMove(SmartDashboard.getNumber("Elevator Point", -4)), 
    //     m_robotClimber
    //   )
    // );

    // Configure the button bindings
    configureButtonBindings();
    configureAutos();
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

    // zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));

    driver.button(3).onTrue(
      new InstantCommand(() -> s_Swerve.zeroGyro())
    );

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

if (false) {

    driver.button(7).whileTrue(
        m_gameCommands.centerATagCommand());

    driver.button(9).and(() -> controllerMode == "Coral").whileTrue(
      m_gameCommands.coralPrepCommand()
    );

    driver.button(9).and(() -> controllerMode == "Algae").whileTrue(
      m_gameCommands.algaePrepCommand()
    );


    operator.button(9).onTrue(
      Commands
      .sequence(
        new InstantCommand(() -> setMode("Coral")),
     //   m_robotCoralPivot.manualPivotMove(-50).until(m_robotCoralPivot::inPosition).asProxy(),
        m_robotFuelPivot.manualPivotMove(-1).until(() -> true).asProxy()     
      )
    );


      // operator.button(1).and(() -> controllerMode == "Coral").whileTrue(
      //   m_gameCommands.coralPivotPositionSetCommand(() -> -825)
      //   //m_gameCommands.manualCoralPivotMove(-700)
      //   //m_robotCoralPivot.testingSpeed(-.1)
      // );

      // operator.button(3).and(() -> controllerMode == "Coral").whileTrue(
      //   m_gameCommands.coralPivotPositionSetCommand(() -> -675)
      //    //m_gameCommands.manualCoralPivotMove(-500)
      //   // m_robotCoralPivot.testingSpeed(.1)
      // );


      // operator.button(2).and(() -> controllerMode == "Coral").whileTrue(
      //   m_gameCommands.coralPivotPositionSetCommand(() -> -100)
      //    //m_gameCommands.manualCoralPivotMove(-100)
      // );
      // operator.button(1).onTrue(
      //   m_gameCommands.algaePivotPositionSetCommand(-1)
         //m_robotAlgaePivot.testingSpeed(-.1)
      //);

      operator.povDown().and(() -> controllerMode == "Coral").onTrue(
        m_gameCommands.climberMoveCommand(-5)
      );

      operator.povLeft().and(() -> controllerMode == "Coral").onTrue(
        m_gameCommands.climberMoveCommand(-35)
      );

      operator.povUp().and(() -> controllerMode == "Coral").onTrue(
        m_gameCommands.climberMoveCommand(-95)
      );

 

      // operator.rightTrigger().and(() -> controllerMode == "Coral").onTrue(
      //  new SequentialCommandGroup(
      //   m_gameCommands.runCoralLaunchCommand(.5).withTimeout(1),
      //   m_gameCommands.coralPivotPositionSetCommand(() -> -50).withTimeout(2),
      //   m_gameCommands.elevatorMoveCommand(-5)
      //  )
      // );

      //operator.leftBumper().onTrue(
      //  //m_robotElevator.bumpPosition(false)
       // new InstantCommand(() -> elevatorOffsetUp())
      //);

      //operator.rightBumper().onTrue(
     //   //m_robotElevator.bumpPosition(true)
      //  new InstantCommand(() -> elevatorOffsetDown())
      //);



      operator.button(4).whileTrue(
        m_gameCommands.fuelPivotPositionSetCommand(() -> -1)
        //m_robotAlgaePivot.testingSpeed(.1)
      );

      operator.povDown().onTrue(
        m_gameCommands.climberMoveCommand(-6)
      );

      operator.povLeft().onTrue(
        m_gameCommands.climberMoveCommand(-45)
      );

      operator.povUp().onTrue(
        m_gameCommands.climberMoveCommand(-75)
      );

      operator.leftTrigger().whileTrue(
        m_gameCommands.runFuelIntakeCommand(0.5)
      );

      operator.rightTrigger().whileTrue(
        m_gameCommands.runFuelLaunchCommand(0.5)
      );


           operator.button(5).whileTrue(
        m_robotClimber.ClimberMove(-.1)
      );

      operator.button(6).whileTrue(
        m_robotClimber.ClimberMove(.1)
      );



    //  operator.button(5).whileTrue(
    //     m_robotTurret.setFireSpeedCommand(.5)
    //   );


      // //down
      // operator.button(5).whileTrue(
      //   m_robotTurret.testingTurretSpeed(.02)
      // );

      // //up
      // operator.button(6).whileTrue(
      //   m_robotTurret.testingTurretSpeed(-.02)
      // );

      operator.button(3).whileTrue(
        m_robotFuel.intake(.1)
      );

      operator.button(2).whileTrue(
        m_robotFuel.launch(.4)
      );

      //down
      operator.button(1).whileTrue(
        m_robotFuelPivot.testingSpeed(.12)
      );

      //up
      operator.button(4).whileTrue(
        m_robotFuelPivot.testingSpeed(-.12)
      );


    

        operator.button(5).whileTrue(
        m_robotLauncher.launch(-.3)
      );

      operator.button(6).whileTrue(
        m_robotLauncher.launch(.3)
      );

      // operator.button(2).whileTrue(
      //   m_robotTurret.setFireSpeedCommand(.5)
      // );




      operator.button(1).whileTrue(
        m_gameCommands.fuelPivotPositionSetCommand(() -> 6.8)
      );

      operator.button(3).whileTrue(
        m_gameCommands.fuelPivotPositionSetCommand(() -> 0)
      );

}

      operator.button(1).whileTrue(
       m_robotFuelPivot.manualPivotMove(6.8)
      );

      operator.button(3).whileTrue(
              m_robotFuelPivot.manualPivotMove(0)
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

  public void setMode(String mode){
    controllerMode = mode;
    SmartDashboard.putString("Controller Mode", mode);
  }
  
  

}