package frc.robot.Parallelogram;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;	
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

public class Parallelogram extends Subsystem{

    //Cargo Positions in motor ticks ****CURRENTLY DUMMY VALUES UNTIL WE GET THE ROBOT TO TEST****
    private static final double bCargoFirstHeight = 1.0;
    private static final double bCargoSecondHeight = 2.0;
    private static final double bCargoThirdHeight = 3.0;

    private static final double fCargoFirstHeight = -3.0;
    private static final double fCargoSecondHeight = -2.0;
    private static final double fCargoThirdHeight = -1.0;

    //Hatch Postions in motor ticks ****CURRENTLY DUMMY VALUES UNTIL WE GET THE ROBOT TO TEST****
    private static final double bHatchFirstHeight = 0.5;
    private static final double bHatchSecondHeight = 1.5;
    private static final double bHatchThirdHeight = 2.5;

    private static final double fHatchFirstHeight = -2.5;
    private static final double fHatchSecondHeight = -1.5;
    private static final double fHatchThirdHeight = -0.5;

    private static final double zeroingSpeed = 0.3;

    private WPI_TalonSRX leftMaster, leftSlave, rightMaster, rightSlave, roller, intakeL, intakeR;

    private DoubleSolenoid rollerDrop, hatchDrop, hatchGrab, ballPusher;

    private DigitalInput bottomHardstopL, bottomHardstopR, topHardstopL, topHardstopR;

    private boolean lastHsWasBottom = true;


    public Parallelogram(){

        leftMaster = new WPI_TalonSRX(Constants.kLeftPMasterID);
        leftSlave = new WPI_TalonSRX(Constants.kLeftPSlaveID);

        rightMaster = new WPI_TalonSRX(Constants.kRightPMasterID);
        rightSlave = new WPI_TalonSRX(Constants.kRightPSlaveID);

        roller = new WPI_TalonSRX(Constants.kRollerID);

        intakeL = new WPI_TalonSRX(Constants.kIntakeL);
        intakeR = new WPI_TalonSRX(Constants.kIntakeR);

        leftMaster.setInverted(Constants.kLeftPInv);
        leftSlave.setInverted(Constants.kLeftPInv);

        intakeL.setInverted(Constants.kIntakeLInv);
        intakeR.setInverted(Constants.kIntakeRInv);

        rollerDrop = new DoubleSolenoid(0, 1);
        hatchGrab = new DoubleSolenoid(2, 5, 1);
        hatchDrop = new DoubleSolenoid(2, 6, 2);
        ballPusher = new DoubleSolenoid(2, 4, 0);

        rightMaster.setInverted(Constants.kRightPInv);
        rightSlave.setInverted(Constants.kRightPInv);

        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);

        leftMaster.setSensorPhase(Constants.kLeftPSensorPhase);
        rightMaster.setSensorPhase(Constants.kRightPSensorPhase);
        leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
		leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
        rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
		rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

        leftSlave.follow(leftMaster);
        
        rightSlave.follow(rightMaster);

        leftMaster.setNeutralMode(NeutralMode.Brake);
        leftSlave.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);
        rightSlave.setNeutralMode(NeutralMode.Brake);

        bottomHardstopL = new DigitalInput(Constants.kBottomHardstopLID);
        bottomHardstopR = new DigitalInput(Constants.kBottomHardstopRID);
        topHardstopL = new DigitalInput(Constants.kTopHardstopLID);
        topHardstopR = new DigitalInput(Constants.kTopHardstopRID);

        /*leftMaster.config_kF(0, Constants.kParalleogramF, 30);
        leftMaster.config_kP(0, Constants.kParalleogramP, 30);
        leftMaster.config_kI(0, Constants.kParalleogramI, 30);
        leftMaster.config_kD(0, Constants.kParalleogramD, 30);
        leftMaster.configMotionCruiseVelocity(Constants.kParalleogramMaxVel, 30);
        leftMaster.configMotionCruiseAcceleration(Constants.kParalleogramMaxAccel, 30);

        rightMaster.config_kF(0, Constants.kParalleogramF, 30);
        rightMaster.config_kP(0, Constants.kParalleogramP, 30);
        rightMaster.config_kI(0, Constants.kParalleogramI, 30);
        rightMaster.config_kD(0, Constants.kParalleogramD, 30);
        rightMaster.configMotionCruiseVelocity(Constants.kParalleogramMaxVel, 30);
        rightMaster.configMotionCruiseAcceleration(Constants.kParalleogramMaxAccel, 30);*/

    }

    @Override
    public void initDefaultCommand() {

        resetEncoders();
        rollerDrop.set(DoubleSolenoid.Value.kReverse);
        hatchDrop.set(DoubleSolenoid.Value.kReverse);
        hatchGrab.set(DoubleSolenoid.Value.kReverse);

    }

    public void zeroBottom(){

        while(!getBottomHSL() && !getBottomHSR()){

            if(!getBottomHSL()){

                leftMaster.set(-zeroingSpeed);

            }
            else{

                leftMaster.set(0);

            }

            if(!getBottomHSR()){

                rightMaster.set(-zeroingSpeed);

            }
            else{

                rightMaster.set(0);

            }

        }

        resetEncoders();    

    }

    public void dropHatch(boolean bool){

        if(bool){

            hatchDrop.set(DoubleSolenoid.Value.kForward);

        }
        else{

            hatchDrop.set(DoubleSolenoid.Value.kReverse);

        }

    }

    public void openClaw(boolean bool){

        if(bool){

            hatchGrab.set(DoubleSolenoid.Value.kForward);

        }
        else{

            hatchGrab.set(DoubleSolenoid.Value.kReverse);

        }

    }

    public void pushBall(boolean bool){

        if(bool){

            ballPusher.set(DoubleSolenoid.Value.kForward);

        }
        else{

            ballPusher.set(DoubleSolenoid.Value.kReverse);

        }

    }

    public void dropRoller(boolean bool){

        if(bool){

            rollerDrop.set(DoubleSolenoid.Value.kForward);

        }
        else{

            rollerDrop.set(DoubleSolenoid.Value.kReverse);

        }

    }

    public void setCur(double Cur){

        leftMaster.set(ControlMode.Current, Cur);
        rightMaster.set(ControlMode.Current, Cur);

    }

    public void setPower(double lSpeed, double rSpeed){

        leftMaster.set(lSpeed);
        rightMaster.set(rSpeed);

    }

    public void setLeftPower(double speed){

        leftMaster.set(speed);

    }

    public void setRightPower(double speed){

        rightMaster.set(speed);

    }

    public void setPos(double Pos){

        
        rightMaster.follow(leftMaster);
        leftMaster.set(ControlMode.Position, Pos);

    }

    public void setVel(double Vel){

        leftMaster.set(ControlMode.Velocity, Vel);
        rightMaster.set(ControlMode.Velocity, Vel);

    }

    public void setMotionMagic(double TargetPos){

       // while(!getBottomHS() && !getTopHS()){

            leftMaster.set(ControlMode.MotionMagic, TargetPos);
            rightMaster.set(ControlMode.MotionMagic, TargetPos);

        //}
        //leftMaster.set(0);
        //rightMaster.set(0);

    }

    public void stopMotors(){

        leftMaster.set(0.0);
        rightMaster.set(0.0);

    }

    public void setRoller(double power){

        roller.set(power);

    }

    public void stopRoller(){

        roller.set(0);

    }

    public void setIntake(double power){

        intakeL.set(power);
        intakeR.set(power);

    }

    public void stopIntake(){

        intakeL.set(0);
        intakeR.set(0);

    }

    public void resetEncoders(){

        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);

    }

    public double getLeftTicks(){

        return leftMaster.getSelectedSensorPosition();

    }

    public double getRightTicks(){

        return rightMaster.getSelectedSensorPosition();

    }

    public double getLeftTicksPer100ms(){

        return leftMaster.getSelectedSensorVelocity();

    }

    public double getRightTicksPer100ms(){

        return rightMaster.getSelectedSensorVelocity();

    }

    public boolean getTopHSL(){

        return topHardstopL.get();

    }

    public boolean getTopHSR(){

        return topHardstopR.get();

    }

    public boolean getBottomHSL(){

        return bottomHardstopL.get();

    }

    public boolean getBottomHSR(){

        return bottomHardstopR.get();

    }

    public double getMotorPower(){

        return leftMaster.getOutputCurrent();

    }

}