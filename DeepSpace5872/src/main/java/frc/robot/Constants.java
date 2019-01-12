package frc.robot;

public class Constants{

    //Drivetrain
    public static final int kLeftDriveMasterID = 0;
    public static final int kLeftDriveSlaveID = 1;
    public static final int kRightDriveMasterID = 2;
    public static final int kRightDriveSlaveID = 3;

    public static final boolean kLeftDriveInv = true;
    public static final boolean kRightDriveInv = false;

    public static final boolean kLeftDriveSensorPhase = true;
    public static final boolean kRightDriveSensorPhase = false;

    public static final int kShifterF = 0;
    public static final int kShifterB = 1;

    public final static int kSensorUnitsPerRotation = 4096;
	
	/**
	 * This is a property of the Pigeon IMU, and should not be changed.
	 */
	public final static double kPigeonUnitsPerRotation = 8192.0;
	
	/**
	 * Set to zero to skip waiting for confirmation.
	 * Set to nonzero to wait and report to DS if action fails.
	 */
	public final static int kTimeoutMs = 30;

	/**
	 * Motor neutral dead-band, set to the minimum 0.1%.
	 */
	public final static double kNeutralDeadband = 0.001;
	
	/**
	 * Base trajectory period to add to each individual 
	 * trajectory point's unique duration.  This can be set
	 * to any value within [0,255]ms.
	 */
	public final static int kBaseTrajPeriodMs = 0;
	
	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop
	 * 	                                    			  kP   kI   kD   kF               Iz    PeakOut */
	public final static Gains kGains_Distanc = new Gains( 0.1, 0.0,  0.0, 0.0,            100,  0.50 );
	public final static Gains kGains_Turning = new Gains( 2.0, 0.0,  4.0, 0.0,            200,  1.00 );
	public final static Gains kGains_Velocit = new Gains( 0.1, 0.0, 20.0, 1023.0/7200.0,  300,  0.50 ); /* measured 6800 velocity units at full motor output */
	public final static Gains kGains_MotProf = new Gains( 0.02, 0.0,  0.2, 1023.0/7200.0,  400,  1.00 ); /* measured 6800 velocity units at full motor output */
	
	/** ---- Flat constants, you should not need to change these ---- */
	/* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
	public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;
	/* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/* Firmware currently supports slots [0, 3] and can be used for either PID Set */
	public final static int SLOT_0 = 0;
	public final static int SLOT_1 = 1;
	public final static int SLOT_2 = 2;
	public final static int SLOT_3 = 3;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Distanc = SLOT_0;
	public final static int kSlot_Turning = SLOT_1;
	public final static int kSlot_Velocit = SLOT_2;
	public final static int kSlot_MotProf = SLOT_3;

    //Parallelogram
    public static final int kLeftPMasterID = 4;
    public static final int kLeftPSlaveAID = 5;
    public static final int kLeftPSlaveBID = 6;
    public static final int kRightPMasterID = 7;
    public static final int kRightPSlaveAID = 8;
    public static final int kRightPSlaveBID = 9;
    
    public static final boolean kLeftPInv = true;
    public static final boolean kRightPInv = false;

    public static final boolean kLeftPSensorPhase = true;
    public static final boolean kRightPSensorPhase = false;

    public static final int kBottomHardstopID = 0;
    public static final int kTopHardstopID = 1;

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