package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotCamera extends SubsystemBase {

  double tagX;
  double tagY;
  double tagArea;
  double tagID;

  double spinAmount;
  double gyro;
  String teamCol;
  NetworkTable table;

  NetworkTableEntry EntryTagX;
  NetworkTableEntry EntryTagY;
  NetworkTableEntry EntryTagArea;
  NetworkTableEntry EntryTagID;

  double tagIDSave = -1;

  public RobotCamera() {
  }

  @Override
  public void periodic() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    EntryTagX = table.getEntry("TagX");
    EntryTagY = table.getEntry("TagY");
    EntryTagArea = table.getEntry("TagArea");
    EntryTagID = table.getEntry("TagID");

    tagX = EntryTagX.getDouble(0.0);
    tagY = EntryTagY.getDouble(0.0);
    tagArea = EntryTagArea.getDouble(0.0);
    tagID = EntryTagID.getDouble(0.0);

    if (tagID != -1) {
      tagIDSave = tagID;
    } else {
      tagID = tagIDSave;
    }

    SmartDashboard.putNumber("Tag X", tagX);
    SmartDashboard.putNumber("Tag Y", tagY);
    SmartDashboard.putNumber("Tag Area", tagArea);
    SmartDashboard.putNumber("Tag ID", tagID);

    teamCol = SmartDashboard.getString("Team Color", "Red");

    gyro = SmartDashboard.getNumber("Gyro", 0);

    SmartDashboard.putString("Team Color", teamCol);

    if (tagID == 5 || tagID == 8 || tagID == 21 || tagID == 24) {

      spinAmount = 90;

    } else if (tagID == 2 || tagID == 11 || tagID == 18 || tagID == 27) {

      spinAmount = -90;

    } else if (tagID == 7 || tagID == 9 || tagID == 10 || tagID == 12 || tagID == 23 || tagID == 25 || tagID == 26
        || tagID == 28) {

      spinAmount = 0;

    } else if (tagID == 1 || tagID == 3 || tagID == 4 || tagID == 6 || tagID == 13 || tagID == 14 || tagID == 15
        || tagID == 16 || tagID == 17 || tagID == 19 || tagID == 20 || tagID == 22 || tagID == 29 || tagID == 30
        || tagID == 31 || tagID == 32) {

      spinAmount = 180;

    } else if (tagID == 42) /* Used only for the testing April Tag */ {

      spinAmount = 0;

    }

    SmartDashboard.putNumber("Spin Amount", spinAmount);
  }
}