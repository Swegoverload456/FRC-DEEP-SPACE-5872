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

import frc.robot.Constants;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static final double wheelDiamterIn = 4.0;
  private static final double gearRatio = 1.0;
  private static final double ticksPerRev = 4096;
  private static final double ticksPerIn = (ticksPerRev * gearRatio) / (wheelDiamterIn * Math.PI);
  private static final double encRefreshRate = 100; //in ms
  private static final double ticksPerInOverSec = (ticksPerIn) / (encRefreshRate * 10);
  private boolean isHighGear = true;

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

    }
    else if(!highGear){

        shifter.set(DoubleSolenoid.Value.kReverse);

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