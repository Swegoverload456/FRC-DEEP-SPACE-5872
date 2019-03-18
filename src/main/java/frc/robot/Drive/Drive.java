package frc.robot.Drive;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

import java.io.File;
import java.io.IOException;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final double wheelDiamterIn = 4.0;
  private final double gearRatio = 1.0;
  private final double ticksPerRev = 4096;
  public final double ticksPerIn = (ticksPerRev * gearRatio) / (wheelDiamterIn * Math.PI);
  public final double inchesPerTicks = (ticksPerRev * wheelDiamterIn * Math.PI);
  private final double encRefreshRate = 100; //in ms
  public final double inchesPerTicksPerSec = 10 / 4096 * (4 * 3.1415926535) / 12;
  public final double ticksPerInPerSec = 10 / ticksPerIn;
  private boolean isHighGear = true;
  private final double maxVel = 144;
  double [] xyz_dps = new double[3];

  private static final String pathName = "example";

  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;

  Trajectory leftTrajectory;
  Trajectory rightTrajectory;
  
  File leftCSV;
  File rightCSV;

  double dt;

  private Notifier m_follower_notifier;

  public final WPI_TalonSRX leftMaster, leftSlave, rightMaster, rightSlave;
  private final DoubleSolenoid shifterL, shifterR, shifterLiftL, shifterLiftR, liftReleaseL, liftReleaseR;
  public final PigeonIMU pigeon;

  public Drive(){

    leftMaster = new WPI_TalonSRX(Constants.kLeftDriveMasterID);
    leftSlave = new WPI_TalonSRX(Constants.kLeftDriveSlaveID);
    rightMaster = new WPI_TalonSRX(Constants.kRightDriveMasterID);
    rightSlave = new WPI_TalonSRX(Constants.kRightDriveSlaveID);

    leftMaster.setInverted(Constants.kLeftDriveInv);
    leftSlave.setInverted(Constants.kLeftDriveInv);
    rightMaster.setInverted(Constants.kRightDriveInv);
    rightSlave.setInverted(Constants.kRightDriveInv);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
    leftMaster.setSensorPhase(Constants.kLeftDriveSensorPhase);
    rightMaster.setSensorPhase(Constants.kRightDriveSensorPhase);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, 30);
		leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 30);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, 30);
		rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 30);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);

    leftMaster.configPeakOutputForward(1, 100);
    rightMaster.configPeakOutputForward(1, 100);
    leftSlave.configPeakOutputReverse(-1, 100);
    rightSlave.configPeakOutputReverse(-1, 100);

    /*leftMaster.config_kF(0, Constants.driveKF, 30);
    rightMaster.config_kF(0, Constants.driveKF, 30);
    leftMaster.config_kP(0, Constants.driveKP, 30);
    rightMaster.config_kP(0, Constants.driveKP, 30);
    leftMaster.config_kI(0, Constants.driveKI, 30);
    rightMaster.config_kI(0, Constants.driveKI, 30);
    leftMaster.config_kD(0, Constants.driveKD, 30);
    rightMaster.config_kD(0, Constants.driveKD, 30);*/

    //rightMaster.config_IntegralZone(0, 300, 30);
		//rightMaster.configClosedLoopPeakOutput(0, 0.75, 30);
		//rightMaster.configAllowableClosedloopError(0, 0, 30);

    //leftMaster.configClosedLoopPeriod(0, 1, 30);
		//leftMaster.configClosedLoopPeriod(1, 1, 30);
    //rightMaster.configClosedLoopPeriod(0, 1, 30);
		//rightMaster.configClosedLoopPeriod(1, 1, 30);

    shifterL = new DoubleSolenoid(1, 0, 7);
    shifterR = new DoubleSolenoid(2, 7, 3);
    shifterLiftL = new DoubleSolenoid(1, 1, 6);
    shifterLiftR = new DoubleSolenoid(0, 6, 7);
    liftReleaseL = new DoubleSolenoid(1, 2, 5);
    liftReleaseR = new DoubleSolenoid(1, 3, 4);

    pigeon = new PigeonIMU(leftSlave);
    pigeon.setFusedHeading(0.0, 30);
    leftSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

    //RemoteSensorSource senSource = RemoteSensorSource.Pigeon_Yaw;

  }

  /*public static File getTrajectoryFile(String name) {

    return new File(Filesystem.getDeployDirectory(),  name + ".pf1.csv");
  
  }

  public static Trajectory getTrajectory(String name) throws IOException{

    return Pathfinder.readFromCSV(getTrajectoryFile(name));

  }

  public void loadPath(String pathName) throws IOException{

    try{

    leftCSV = new File("/home/lvuser/deploy/output/" + pathName + ".left.pf1.csv");
    rightCSV = new File("/home/lvuser/deploy/output/" + pathName + ".right.pf1.csv");
    leftTrajectory = getTrajectory(pathName + ".right");
    rightTrajectory = getTrajectory(pathName + ".left");

      dt = leftTrajectory.get(0).dt;

    }
    catch(IOException e){

      e.printStackTrace();

    }
      //System.out.println("CSV has been locked and loaded");

  }

  public void autoInit(){

    m_left_follower = new EncoderFollower(leftTrajectory);
    m_right_follower = new EncoderFollower(rightTrajectory);

    m_left_follower.configureEncoder(getLeftTicks(), 4096, 4);
    // You must tune the PID values on the following line!
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / maxVel, 0);

    m_right_follower.configureEncoder(getRightTicks(), 4096, 4);
    // You must tune the PID values on the following line!
    m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / maxVel, 0);
    
    m_follower_notifier = new Notifier(this::followPath);
    m_follower_notifier.startPeriodic(leftTrajectory.get(0).dt);

  }

  public void followPath() {
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      m_follower_notifier.stop();
    } else {
      double left_speed = m_left_follower.calculate(getLeftTicks());
      double right_speed = m_right_follower.calculate(getRightTicks());
      //double heading = getHeading();
      //double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
        //double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      //double turn =  0.8 * (-1.0/80.0) * heading_difference;
      leftMaster.set(left_speed);
      rightMaster.set(right_speed);
    }
  }*/

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    resetEncoders();
    resetHeading();
    shifterL.set(DoubleSolenoid.Value.kReverse);
    shifterR.set(DoubleSolenoid.Value.kReverse);
    shifterLiftL.set(DoubleSolenoid.Value.kForward);
    shifterLiftR.set(DoubleSolenoid.Value.kForward);
    liftReleaseL.set(DoubleSolenoid.Value.kForward);
    liftReleaseR.set(DoubleSolenoid.Value.kForward);

  }

  public void driveStraight(double power){

      if(getHeading() > 0.5){

        setPower(power + ((getHeading()) / 75), power);

      }
      else if(getHeading() < -0.5){

        setPower(power, power + ((getHeading()) / 75));

      }
      else{

        setPower(power, power);

      }

  }

  public void setPower(double leftPower, double rightPower){

    leftMaster.set(leftPower);
    rightMaster.set(rightPower);

  }
  public void setCurrent(double leftCur, double rightCur){

    leftMaster.set(ControlMode.Current, leftCur);
    rightMaster.set(ControlMode.Current, rightCur);

  }
  public void setPos(double leftPos, double rightPos){

    leftMaster.set(ControlMode.Position, leftPos * ticksPerIn);
    rightMaster.set(ControlMode.Position, rightPos * ticksPerIn);

  }
  public void setVel(double leftVel, double rightVel){

    leftMaster.set(ControlMode.Velocity, leftVel);
    rightMaster.set(ControlMode.Velocity, rightVel);

  }

  public void stopMotors(){

    leftMaster.set(0);
    rightMaster.set(0);

  }

  public void resetEncoders(){

    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);

  }

  public void turn(double targetAngle){

    resetHeading();

    //Assuming that our max turning vel is 9.15 ft/sec
    //equation for finding this was kp(180[max error]) = 109.8 in/sec
    double kP = 0.61;
    double threshold = 1;

    double error = targetAngle - getRelativeHeading();

    while(error > threshold || error < -threshold){

      setVel(kP*error, kP*error);

    }

    stopMotors();

  }

  public double getRelativeHeading(){

    double angle = 0;

    if(getHeading() > 180){

      angle = getHeading() - 360;

    }
    else{

      angle = getHeading();

    }

    return angle;

  }

  public void resetHeading(){

    pigeon.setFusedHeading(0, 30);

  }

  public void setShifterState(boolean lowGear){

    if(lowGear){

        shifterL.set(DoubleSolenoid.Value.kForward);
        shifterR.set(DoubleSolenoid.Value.kForward);
        isHighGear = false;

    }
    else if(!lowGear){

        shifterL.set(DoubleSolenoid.Value.kReverse);
        shifterR.set(DoubleSolenoid.Value.kReverse);
        isHighGear = true;

    }

  }

  public void engageLift(boolean bool){

    if(!bool){

        shifterLiftL.set(DoubleSolenoid.Value.kForward);
        shifterLiftR.set(DoubleSolenoid.Value.kForward);

    }
    else if(bool){

        shifterLiftL.set(DoubleSolenoid.Value.kReverse);
        shifterLiftR.set(DoubleSolenoid.Value.kReverse);
        shifterL.set(DoubleSolenoid.Value.kForward);
        shifterR.set(DoubleSolenoid.Value.kForward);

    }

  }

  public void releaseLift(boolean bool){

    if(!bool){

        liftReleaseL.set(DoubleSolenoid.Value.kForward);
        liftReleaseR.set(DoubleSolenoid.Value.kForward);

    }
    else if(bool){

        liftReleaseL.set(DoubleSolenoid.Value.kReverse);
        liftReleaseR.set(DoubleSolenoid.Value.kReverse);

    }

  }

  public double getHeading(){

    PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();

    return pigeon.getFusedHeading(fusionStatus);

  }

  public int getLeftTicks(){

    return (int)leftMaster.getSelectedSensorPosition();

  }
  public double getLeftInches(){

    return leftMaster.getSelectedSensorPosition() / ticksPerIn;

  }

  public int getRightTicks(){

    return (int) rightMaster.getSelectedSensorPosition();

  }

  public double getRightInches(){

    return rightMaster.getSelectedSensorPosition() / ticksPerIn;

  }

  public double getLeftVelTicksPerSec(){

    return leftMaster.getSelectedSensorVelocity() * 10;

  }

  public double getLeftVelInchesPerSec(){

    return leftMaster.getSelectedSensorVelocity() * 10 / 4096 * (4 * 3.1415926535) / 12;

  }

  public double getRightVelTicksPerSec(){

    return rightMaster.getSelectedSensorVelocity() * 10;

  }
  public double getRightVelInchesPerSec(){

    return rightMaster.getSelectedSensorVelocity() * 10 / 4096 * (4 * 3.1415926535) / 12;

  }

  public boolean getShifterState(){

    return isHighGear;

  }

}
