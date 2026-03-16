package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.GameCommands;
import frc.robot.subsystems.RobotIntakePivot;
import frc.robot.subsystems.RobotClimber;
import frc.robot.Constants;

public class Autos {

  private final GameCommands m_gameCommands;

  private final RobotIntakePivot m_robotFuelPivot;

  private final RobotClimber m_robotClimber;

  public Autos(
    GameCommands gameCommands, RobotClimber robotElevator, RobotIntakePivot robotFuelPivot
  ) {
    m_gameCommands = gameCommands;
    m_robotClimber = robotElevator;
    m_robotFuelPivot = robotFuelPivot;
  }

  public Command autoCommand1() {
    return Commands
      .sequence(
        m_gameCommands.climberMoveCommand(-7).until(() -> m_robotClimber.inPosition()).asProxy(),
     
        m_gameCommands.driveCommand(() -> .4, () -> 0, () -> 0).withTimeout(1.6),
      m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0),
      
      
      m_gameCommands.driveRotateUntilCommand(() -> 300).withTimeout(Constants.AutoConstants.timeSpinning),
      m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0),


        // m_gameCommands.driveRotateUntilCommand(() -> 60).withTimeout(Constants.AutoConstants.timeSpinning),
        // m_gameCommands.driveCommand(() -> Constants.AutoConstants.speedDriving, () -> 0, () -> 0).withTimeout(Constants.AutoConstants.timeForward),
        // m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(1),
         m_gameCommands.coralPrepCommand().withTimeout(Constants.AutoConstants.timeCentering),
         m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0),

         m_gameCommands.climberMoveCommand(-95).withTimeout(3.5)/*.until(() -> m_robotElevator.inPosition())*/.asProxy(),
         //m_gameCommands.runCoralLaunchCommand(Constants.AutoConstants.speedLaunching).withTimeout(Constants.AutoConstants.timeLaunching),
         m_gameCommands.climberMoveCommand(Constants.AutoConstants.positionEleBottom).until(() -> m_robotClimber.inPosition()).asProxy()

        // m_gameCommands.elevatorMoveCommand(-7).withTimeout(1).asProxy(),
        // m_gameCommands.driveRotateUntilCommand(() -> 300).withTimeout(Constants.AutoConstants.timeSpinning),
        // m_gameCommands.driveCommand(() -> Constants.AutoConstants.speedDriving, () -> 0, () -> 0).withTimeout(Constants.AutoConstants.timeForward),
        // m_gameCommands.coralPrepCommand().withTimeout(Constants.AutoConstants.timeCentering),
        // m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0),
        // m_gameCommands.elevatorMoveCommand(Constants.AutoConstants.positionEleTop).withTimeout(Constants.AutoConstants.timeEleMoving).asProxy(),
        // m_gameCommands.coralPivotPositionSetCommand(() -> Constants.AutoConstants.positionCoralPivot).withTimeout(Constants.AutoConstants.timePivoting).asProxy(),
        // m_gameCommands.runCoralLaunchCommand(Constants.AutoConstants.speedLaunching).withTimeout(Constants.AutoConstants.timeLaunching),
        // m_gameCommands.coralPivotPositionSetCommand(() -> -50).withTimeout(Constants.AutoConstants.timePivoting).asProxy(),
        // m_gameCommands.elevatorMoveCommand(Constants.AutoConstants.positionEleBottom).withTimeout(Constants.AutoConstants.timeEleMoving).asProxy(),
        // m_gameCommands.algaePivotPositionSetCommand(() -> Constants.AutoConstants.positionAlgaePivot).withTimeout(Constants.AutoConstants.timePivoting).asProxy(),
        // m_gameCommands.runAlgaeIntakeCommand(Constants.AutoConstants.speedIntaking).withTimeout(Constants.AutoConstants.timeIntaking)
      )
      .withName("autoCommand");
  }
  public Command autoCommand2() {
    return Commands
      .sequence(
        m_gameCommands.driveCommand(() -> .5, () -> 0, () -> 0).withTimeout(0.5),
        m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0),
         m_gameCommands.climberMoveCommand(-7).until(() -> m_robotClimber.inPosition()).asProxy(),
     
         m_gameCommands.coralPrepCommand().withTimeout(4.5),
         m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0.05),
         m_gameCommands.climberMoveCommand(-95).withTimeout(3.5)/*.until(() -> m_robotElevator.inPosition())*/.asProxy(),
         //m_gameCommands.runCoralLaunchCommand(Constants.AutoConstants.speedLaunching).withTimeout(Constants.AutoConstants.timeLaunching),
         m_gameCommands.climberMoveCommand(Constants.AutoConstants.positionEleBottom).until(() -> m_robotClimber.inPosition()).asProxy()
      )
      .withName("autoCommand");
  }

public Command autoCommand3() {
  return Commands
    .sequence(



    m_gameCommands.climberMoveCommand(-7).until(() -> m_robotClimber.inPosition()).asProxy(),
     
    m_gameCommands.driveCommand(() -> .5, () -> 0, () -> 0).withTimeout(1.6),
  m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0),
  
  
  m_gameCommands.driveRotateUntilCommand(() -> 60).withTimeout(Constants.AutoConstants.timeSpinning),
  m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0),


    // m_gameCommands.driveRotateUntilCommand(() -> 60).withTimeout(Constants.AutoConstants.timeSpinning),
    // m_gameCommands.driveCommand(() -> Constants.AutoConstants.speedDriving, () -> 0, () -> 0).withTimeout(Constants.AutoConstants.timeForward),
    // m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(1),
     m_gameCommands.coralPrepCommand().withTimeout(Constants.AutoConstants.timeCentering),
     m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0),

     m_gameCommands.climberMoveCommand(-95).withTimeout(3.3)/*.until(() -> m_robotElevator.inPosition())*/.asProxy(),
     //m_gameCommands.runCoralLaunchCommand(Constants.AutoConstants.speedLaunching).withTimeout(Constants.AutoConstants.timeLaunching),
     m_gameCommands.climberMoveCommand(Constants.AutoConstants.positionEleBottom).until(() -> m_robotClimber.inPosition()).asProxy()



     //m_gameCommands.elevatorMoveCommand(-7).until(() -> m_robotElevator.inPosition()).asProxy(),
     //m_gameCommands.driveRotateUntilCommand(() -> 240).withTimeout(Constants.AutoConstants.timeSpinning),
     // m_gameCommands.driveRotateUntilCommand(() -> 60).withTimeout(Constants.AutoConstants.timeSpinning),
     // m_gameCommands.driveCommand(() -> Constants.AutoConstants.speedDriving, () -> 0, () -> 0).withTimeout(Constants.AutoConstants.timeForward),
     // m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(1),
     // m_gameCommands.coralPrepCommand().withTimeout(Constants.AutoConstants.timeCentering),
     // m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0)
     // m_gameCommands.elevatorMoveCommand(-20).withTimeout(2)/*.until(() -> m_robotElevator.inPosition())*/.asProxy(),
     // m_gameCommands.coralPivotPositionSetCommand(() -> -825).withTimeout(3)/*.until(() -> m_robotCoralPivot.inPosition())*/.asProxy(),
     // m_gameCommands.runCoralLaunchCommand(Constants.AutoConstants.speedLaunching).withTimeout(Constants.AutoConstants.timeLaunching),
     // m_gameCommands.coralPivotPositionSetCommand(() -> -50).withTimeout(2)/*.until(() -> m_robotCoralPivot.inPosition())*/.asProxy(),
     // m_gameCommands.elevatorMoveCommand(Constants.AutoConstants.positionEleBottom).until(() -> m_robotElevator.inPosition()).asProxy()

      // m_gameCommands.elevatorMoveCommand(-7).withTimeout(1).asProxy(),
      // m_gameCommands.driveRotateUntilCommand(() -> 60).withTimeout(Constants.AutoConstants.timeSpinning),
      // m_gameCommands.driveCommand(() -> Constants.AutoConstants.speedDriving, () -> 0, () -> 0).withTimeout(Constants.AutoConstants.timeForward),
      // m_gameCommands.coralPrepCommand().withTimeout(Constants.AutoConstants.timeCentering),
      // m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0),
      // m_gameCommands.elevatorMoveCommand(Constants.AutoConstants.positionEleTop).withTimeout(Constants.AutoConstants.timeEleMoving).asProxy(),
      // m_gameCommands.coralPivotPositionSetCommand(() -> Constants.AutoConstants.positionCoralPivot).withTimeout(Constants.AutoConstants.timePivoting).asProxy(),
      // m_gameCommands.runCoralLaunchCommand(Constants.AutoConstants.speedLaunching).withTimeout(Constants.AutoConstants.timeLaunching),
      // m_gameCommands.coralPivotPositionSetCommand(() -> -50).withTimeout(Constants.AutoConstants.timePivoting).asProxy(),
      // m_gameCommands.elevatorMoveCommand(Constants.AutoConstants.positionEleBottom).withTimeout(Constants.AutoConstants.timeEleMoving).asProxy(),
      // m_gameCommands.algaePivotPositionSetCommand(() -> Constants.AutoConstants.positionAlgaePivot).withTimeout(Constants.AutoConstants.timePivoting).asProxy(),
      // m_gameCommands.runAlgaeIntakeCommand(Constants.AutoConstants.speedIntaking).withTimeout(Constants.AutoConstants.timeIntaking)
    )
    .withName("autoCommand");
}

public Command autoCommandTEST() {
  return Commands
    .sequence(
     m_gameCommands.driveCommand(() -> .5, () -> 0, () -> 0).withTimeout(0.5),
      m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0)
    //   m_gameCommands.elevatorMoveCommand(-7).until(() -> m_robotElevator.inPosition()).asProxy(),
      // m_gameCommands.driveRotateUntilCommand(() -> 60).withTimeout(Constants.AutoConstants.timeSpinning),
      // m_gameCommands.driveCommand(() -> Constants.AutoConstants.speedDriving, () -> 0, () -> 0).withTimeout(Constants.AutoConstants.timeForward),
      // m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(1),
    //   m_gameCommands.coralPrepCommand().withTimeout(Constants.AutoConstants.timeCentering),
    //   m_gameCommands.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0.05),
    //   m_gameCommands.elevatorMoveCommand(-95).withTimeout(4)/*.until(() -> m_robotElevator.inPosition())*/.asProxy(),
    //   m_gameCommands.coralPivotPositionSetCommand(() -> -825).withTimeout(3)/*.until(() -> m_robotCoralPivot.inPosition())*/.asProxy(),
    //   m_gameCommands.runCoralLaunchCommand(Constants.AutoConstants.speedLaunching).withTimeout(Constants.AutoConstants.timeLaunching),
   //    m_gameCommands.coralPivotPositionSetCommand(() -> -50).withTimeout(2)/*.until(() -> m_robotCoralPivot.inPosition())*/.asProxy(),
   //    m_gameCommands.elevatorMoveCommand(Constants.AutoConstants.positionEleBottom).until(() -> m_robotElevator.inPosition()).asProxy()
    )
    .withName("autoCommand");
}

}