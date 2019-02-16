package frc.robot;

public class Constants{

    //Drivetrain
    public static final int kLeftDriveMasterID = 7;
    public static final int kLeftDriveSlaveID = 2;
    public static final int kRightDriveMasterID = 6;
    public static final int kRightDriveSlaveID = 10;

    public static final boolean kLeftDriveInv = false;
    public static final boolean kRightDriveInv = true;

    public static final boolean kLeftDriveSensorPhase = true;
    public static final boolean kRightDriveSensorPhase = true;

    public static final int kShifterLF = 0;
    public static final int kShifterLB = 1;
    public static final int kShifterRF = 7;
    public static final int kShifterRB = 0;
    public static final int kShifterLiftLF = 6;
    public static final int kShifterLiftLB = 1;
    public static final int kShifterLiftRF = 5;
    public static final int kShifterLiftRB = 2;
    public static final int kLiftReleaseRF = 4;
    public static final int kLiftReleaseRB = 3;
    public static final int kLiftReleaseLF = 7;
    public static final int kLiftReleaseLB = 6;

    public static final double driveKF = 0.10969333047;
    public static final double driveKP = 0;
    public static final double driveKI = 0;
    public static final double driveKD = 0;

    //Parallelogram
    public static final int kLeftPMasterID = 4;
    public static final int kLeftPSlaveID = 8;
    public static final int kRightPMasterID = 3;
    public static final int kRightPSlaveID = 0;

    public static final int kRollerID = 5;
    public static final int kIntakeL = 9;
    public static final int kIntakeR = 1;

    public static final boolean kIntakeLInv = false;
    public static final boolean kIntakeRInv = true;
    
    public static final boolean kLeftPInv = true;
    public static final boolean kRightPInv = false;

    public static final boolean kLeftPSensorPhase = true;
    public static final boolean kRightPSensorPhase = true;

    public static final int kBottomHardstopLID = 1;
    public static final int kBottomHardstopRID = 0;

    public static final int kTopHardstopLID = 2;
    public static final int kTopHardstopRID = 3;

    public static final double kParallelogramF = 0.000;
    public static final double kParallelogramP = 2.5575e-4;
    public static final double kParallelogramI = 0;
    public static final double kParallelogramD = 0;
    public static final double kParallelogramMaxVel = 30000;
    public static final double kParallelogramMaxAccel = 30000;

    //Led
    public static final int kBlinkinID = 0;

    public static final double black = 0.99;
    public static final double darkGray = 0.97;
    public static final double gray = 0.95;
    public static final double white = 0.93;
    public static final double violet = 0.91;
    public static final double blueViolet = 0.89;
    public static final double blue = 0.87;
    public static final double darkBlue = 0.85;
    public static final double skyBlue = 0.83;
    public static final double aqua = 0.81;
    public static final double blueGreen = 0.79;
    public static final double green = 0.77;
    public static final double darkGreen = 0.75;
    public static final double lime = 0.73;
    public static final double lawnGreen = 0.71;
    public static final double yellow = 0.69;
    public static final double gold = 0.67;
    public static final double orange = 0.65;
    public static final double redOrange = 0.63;
    public static final double red = 0.61;
    public static final double darkRed = 0.59;
    public static final double hotPink = 0.57;

    //Random Physics
    public static final double gravity = -9.81;

}