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
  NetworkTable table2;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tid;
  

  double tagIDSave = -1;


  public RobotCamera() {
  }

  @Override
  public void periodic() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    table2 = NetworkTableInstance.getDefault().getTable("limelight-two");
    tx = table2.getEntry("tx");
    ty = table2.getEntry("ty");
    ta = table2.getEntry("ta");
    tid = table2.getEntry("tid");
   

    tagX = tx.getDouble(0.0);
    tagY = ty.getDouble(0.0);
    tagArea = ta.getDouble(0.0);
    tagID = tid.getDouble(0.0);
    

    if (tagID != -1){
      tagIDSave = tagID;
    } else {
      tagID = tagIDSave;}

    SmartDashboard.putNumber("Tag X", tagX);
    SmartDashboard.putNumber("Tag Y", tagY);
    SmartDashboard.putNumber("Tag Area", tagArea);
    SmartDashboard.putNumber("Tag ID", tagID);
   

    teamCol = SmartDashboard.getString("Team Color", "r");

    gyro = SmartDashboard.getNumber("Gyro", 0);

    SmartDashboard.putString("Team Color", teamCol);

    if (tagID == 5 || tagID == 8 || tagID == 21 || tagID == 24) {
    
        spinAmount = 90;

    } else if (tagID == 2 || tagID == 11 || tagID == 18 || tagID == 27){

        spinAmount = -90;
    
    } else if (tagID == 7 || tagID == 9 || tagID == 10 || tagID == 12 || tagID == 23 || tagID == 25 || tagID == 26 || tagID == 28){
    
        spinAmount = 0;
    
    } else if (tagID == 1 || tagID == 3 || tagID == 4 || tagID == 6 || tagID == 13 || tagID == 14 || tagID == 15 || tagID == 16 || tagID == 17 || tagID == 19 || tagID == 20 || tagID == 22 || tagID == 29 || tagID == 30 || tagID == 31 || tagID == 32) {
    
        spinAmount = 180;
    
    } else if (tagID == 42) /* Used only for the testing April Tag */{

        spinAmount = 0;

    }

    SmartDashboard.putNumber("Spin Amount", spinAmount);
  }
}