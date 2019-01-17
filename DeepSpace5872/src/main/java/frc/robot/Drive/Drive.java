package frc.robot.Drive;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.PigeonIMU;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

import frc.robot.Constants;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static final double wheelDiamterIn = 4.0;
  private static final double gearRatio = 1.0;
  private static final int ticksPerRev = 4096;
  private static final double ticksPerIn = (ticksPerRev * gearRatio) / (wheelDiamterIn * Math.PI);
  private static final double encRefreshRate = 100; //in ms
  private static final double ticksPerInOverSec = (ticksPerIn) / (encRefreshRate * 10);
  private static final int maxVel = 17;
  private boolean isHighGear = true;

  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;

  private Notifier m_follower_notifier;

  private final WPI_TalonSRX leftMaster, leftSlave, rightMaster, rightSlave;
  private final DoubleSolenoid shifter;
  private final PigeonIMU pigeon;

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
    leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);

    shifter = new DoubleSolenoid(Constants.kShifterF, Constants.kShifterB);

    pigeon = new PigeonIMU(leftSlave);
    leftSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    resetEncoders();
    resetHeading();

  }

  public void initPath(String pathName){

    Trajectory left_trajectory = PathfinderFRC.getTrajectory(pathName + ".left");
    Trajectory right_trajectory = PathfinderFRC.getTrajectory(pathName + ".right");

    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);

    m_left_follower.configureEncoder(leftMaster.getSelectedSensorPosition(), ticksPerRev, wheelDiamterIn);
    // You must tune the PID values on the following line!
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / maxVel, 0);

    m_right_follower.configureEncoder(rightMaster.getSelectedSensorPosition(), ticksPerRev, wheelDiamterIn);
    // You must tune the PID values on the following line!
    m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / maxVel, 0);
    
    m_follower_notifier = new Notifier(this::followPath);
    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
    
  }

  private void followPath() {
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      m_follower_notifier.stop();
    } else {
      double left_speed = m_left_follower.calculate(leftMaster.getSelectedSensorPosition());
      double right_speed = m_right_follower.calculate(rightMaster.getSelectedSensorPosition());
      double heading = pigeon.getAbsoluteCompassHeading();
      double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn =  0.8 * (-1.0/80.0) * heading_difference;
      leftMaster.set(left_speed + turn);
      rightMaster.set(right_speed - turn);
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

    leftMaster.set(ControlMode.Velocity, leftVel * ticksPerInOverSec);
    rightMaster.set(ControlMode.Velocity, rightVel * ticksPerInOverSec);

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

    pigeon.setCompassAngle(0);

  }

  public void setShifterState(boolean highGear){

    if(highGear){

        shifter.set(DoubleSolenoid.Value.kForward);
        isHighGear = true;

    }
    else if(!highGear){

        shifter.set(DoubleSolenoid.Value.kReverse);
        isHighGear = false;

    }
    else{

        shifter.set(DoubleSolenoid.Value.kOff);

    }

  }

  public double getHeading(){

    return pigeon.getAbsoluteCompassHeading();

  }

  public double getLeftTicks(){

    return leftMaster.getSelectedSensorPosition();

  }
  public double getLeftInches(){

    return leftMaster.getSelectedSensorPosition() * ticksPerIn;

  }

  public double getRightTicks(){

    return rightMaster.getSelectedSensorPosition();

  }

  public double getRightInches(){

    return rightMaster.getSelectedSensorPosition() * ticksPerIn;

  }

  public double getLeftVelTicksPerSec(){

    return leftMaster.getSelectedSensorVelocity() * 10;

  }

  public double getLeftVelInchesPerSec(){

    return leftMaster.getSelectedSensorVelocity() * (wheelDiamterIn * Math.PI * 10);

  }

  public double getRightVelTicksPerSec(){

    return rightMaster.getSelectedSensorVelocity() * 10;

  }
  public double getRightVelInchesPerSec(){

    return rightMaster.getSelectedSensorVelocity() * (wheelDiamterIn * Math.PI * 10);

  }

  public boolean getShifterState(){

    return isHighGear;

  }

}