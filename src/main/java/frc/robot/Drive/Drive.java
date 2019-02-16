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
/*import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;*/

import frc.robot.Constants;

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
  private final double encRefreshRate = 100; //in ms
  public final double inchesPerTicksPerSec = 10 / 4096 * (4 * 3.1415926535) / 12;
  public final double ticksPerInPerSec = 10 / ticksPerIn;
  private boolean isHighGear = true;
  private final double maxVel = 180;

  private static final String k_path_name = "example";

  //private EncoderFollower m_left_follower;
  //private EncoderFollower m_right_follower;
  
  private Notifier m_follower_notifier;

  private final WPI_TalonSRX leftMaster, leftSlave, rightMaster, rightSlave;
  private final DoubleSolenoid shifterL, shifterR, shifterLiftL, shifterLiftR, liftReleaseL, liftReleaseR;
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

    leftMaster.config_kF(0, Constants.driveKF, 30);
    rightMaster.config_kF(0, Constants.driveKF, 30);
    leftMaster.config_kP(0, Constants.driveKP, 30);
    rightMaster.config_kP(0, Constants.driveKP, 30);
    leftMaster.config_kI(0, Constants.driveKI, 30);
    rightMaster.config_kI(0, Constants.driveKI, 30);
    leftMaster.config_kD(0, Constants.driveKD, 30);
    rightMaster.config_kD(0, Constants.driveKD, 30);

    rightMaster.config_IntegralZone(0, 300, 30);
		rightMaster.configClosedLoopPeakOutput(0, 0.75, 30);
		rightMaster.configAllowableClosedloopError(0, 0, 30);

    leftMaster.configClosedLoopPeriod(0, 1, 30);
		leftMaster.configClosedLoopPeriod(1, 1, 30);
    rightMaster.configClosedLoopPeriod(0, 1, 30);
		rightMaster.configClosedLoopPeriod(1, 1, 30);

    shifterL = new DoubleSolenoid(1, Constants.kShifterLF, Constants.kShifterLB);
    shifterR = new DoubleSolenoid(Constants.kShifterRF, Constants.kShifterRB);
    shifterLiftL = new DoubleSolenoid(1, Constants.kShifterLiftLF, Constants.kShifterLiftLB);
    shifterLiftR = new DoubleSolenoid(1, Constants.kShifterLiftRF, Constants.kShifterLiftRB);
    liftReleaseL = new DoubleSolenoid(Constants.kLiftReleaseLF, Constants.kLiftReleaseLB);
    liftReleaseR = new DoubleSolenoid(1, Constants.kLiftReleaseRF, Constants.kLiftReleaseRB);

    pigeon = new PigeonIMU(leftSlave);
    leftSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

  }

  /*public void autoInit(){

    Trajectory left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left");
    Trajectory right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");

    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);

    m_left_follower.configureEncoder(m_left_encoder.get(), ticksPerRev, wheelDiamterIn);
    // You must tune the PID values on the following line!
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);

    m_right_follower.configureEncoder(m_right_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!
    m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);
    
    m_follower_notifier = new Notifier(this::followPath);
    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);

  }
*/
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

    pigeon.setCompassAngle(0);

  }

  public void setShifterState(boolean lowGear){

    if(lowGear){

        shifterL.set(DoubleSolenoid.Value.kForward);
        shifterR.set(DoubleSolenoid.Value.kForward);

    }
    else if(!lowGear){

        shifterL.set(DoubleSolenoid.Value.kReverse);
        shifterR.set(DoubleSolenoid.Value.kReverse);

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
