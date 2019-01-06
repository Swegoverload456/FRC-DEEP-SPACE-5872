package frc.robot.Vision;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight{

    int ledState;

    public Limelight(){
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tled = table.getEntry("ledMode");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double targetPresent = tv.getDouble(0.0);
    
    public double getXPos(){

        return x;

    }
    public double getYPos(){

        return y;

    }
    public double getTargArea(){

        return area;

    }
    public boolean getTargetPresent(){
        
        boolean target = false;

        if(targetPresent == 1){

            target = true;

        }
        else if(targetPresent == 0){

            target = false;

        }

        return target;

    }
    
}