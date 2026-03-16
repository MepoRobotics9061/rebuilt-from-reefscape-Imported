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
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tid;
  

  double tagIDSave = -1;


  public RobotCamera() {
  }

  // public Command centerATagCommand(){
  //   return driveCommand(0, tagX/20, gyro - spinAmount);
  // }

  // public Command coralPrepCommand(){
  //   return driveCommand((10 - tagArea)/30, tagX/20,  gyro - spinAmount);
  // }

  @Override
  public void periodic() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tid = table.getEntry("tid");
   

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

    if (tagID == 1) {
    
        spinAmount = 45;
      
    } else if (tagID == 2) {
     
        spinAmount = 315;
     
    } else if (tagID == 3) {
      
        spinAmount = 270;
      
    } else if (tagID == 4 || tagID == 5 || tagID == 7 ) {
     
        spinAmount = 180;
      
    }  else if ( tagID == 21) {
  
        spinAmount = 0;
     
    }  else if (tagID == 6  ) {
      
        spinAmount = 240;
     
    }else if (tagID == 22) {
      
        spinAmount = 60;
     
    }
    
    else if (tagID == 8 ) {
   
        spinAmount = 120;
     
    } 
    else if (tagID == 12 ) {
   
      spinAmount = 315;
   
  } 
    else if (tagID == 20) {
     
        spinAmount = 300;
      
    } 
    else if (tagID == 10 ) { 
        spinAmount = 0;  
    } 
    else if (tagID == 14 || tagID == 15 || tagID == 18) {
     
        spinAmount = 180;
    
    } 

    else if (tagID == 9  ) {
     
        spinAmount = 60;
   
    } 
    else if (tagID == 19) {
      
        spinAmount = 240;
      
    } 
    else if (tagID == 11 ) {
      
        spinAmount = 300;
      
    } 
    else if ( tagID == 17) {
      
        spinAmount = 120;
      
    } 
    else if (tagID == 12) {
     
        spinAmount = 315;
      
    } else if (tagID == 13) {
     
        spinAmount = 45;
      
    } else if (tagID == 16) {
       
          spinAmount = 270;
        
      //Used only for our April Tag (#42)
    } else if (tagID == 42) {
      spinAmount = 0;
    }

    SmartDashboard.putNumber("Spin Amount", spinAmount);
  }
}