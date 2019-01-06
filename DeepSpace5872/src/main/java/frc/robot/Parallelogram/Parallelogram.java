package frc.robot.Parallelogram;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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

    private WPI_TalonSRX leftMaster, leftSlaveA, leftSlaveB, rightMaster, rightSlaveA, rightSlaveB;

    private DigitalInput bottomHardstop, topHardstop;

    private boolean lastHsWasBottom = true;


    public Parallelogram(){

        leftMaster = new WPI_TalonSRX(Constants.kLeftPMasterID);
        leftSlaveA = new WPI_TalonSRX(Constants.kLeftPSlaveAID);
        leftSlaveB = new WPI_TalonSRX(Constants.kLeftPSlaveBID);

        rightMaster = new WPI_TalonSRX(Constants.kRightPMasterID);
        rightSlaveA = new WPI_TalonSRX(Constants.kRightPSlaveAID);
        rightSlaveB = new WPI_TalonSRX(Constants.kRightPSlaveBID);

        leftMaster.setInverted(Constants.kLeftPInv);
        leftSlaveA.setInverted(Constants.kLeftPInv);
        leftSlaveB.setInverted(Constants.kLeftPInv);

        rightMaster.setInverted(Constants.kRightPInv);
        rightSlaveA.setInverted(Constants.kRightPInv);
        rightSlaveB.setInverted(Constants.kRightPInv);

        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);

        leftMaster.setSensorPhase(Constants.kLeftPSensorPhase);
        rightMaster.setSensorPhase(Constants.kRightPSensorPhase);

        leftSlaveA.follow(leftMaster);
        leftSlaveB.follow(leftMaster);
        
        rightSlaveA.follow(rightMaster);
        rightSlaveB.follow(rightMaster);

        leftMaster.setNeutralMode(NeutralMode.Coast);
        leftSlaveA.setNeutralMode(NeutralMode.Coast);
        leftSlaveB.setNeutralMode(NeutralMode.Coast);
        rightMaster.setNeutralMode(NeutralMode.Coast);
        rightSlaveA.setNeutralMode(NeutralMode.Coast);
        rightSlaveB.setNeutralMode(NeutralMode.Coast);

        bottomHardstop = new DigitalInput(Constants.kBottomHardstopID);
        topHardstop = new DigitalInput(Constants.kTopHardstopID);

    }

    @Override
    public void initDefaultCommand() {

        resetEncoders();

    }

    public void zeroBottom(){

        if(!getBottomHS()){

            setPower(-zeroingSpeed);

        }
        else{

            stopMotors();

        }

    }
    public void zeroTop(){

        if(!getTopHS()){

            setPower(zeroingSpeed);

        }
        else{

            stopMotors();

        }

    }

    public void setCur(double Cur){

        leftMaster.set(ControlMode.Current, Cur);
        rightMaster.set(ControlMode.Current, Cur);

    }

    public void setPower(double Speed){

        leftMaster.set(Speed);
        rightMaster.set(Speed);

    }

    public void setPos(double Pos){

        leftMaster.set(ControlMode.Position, Pos);
        rightMaster.set(ControlMode.Position, Pos);

    }

    public void setVel(double Vel){

        leftMaster.set(ControlMode.Velocity, Vel);
        rightMaster.set(ControlMode.Velocity, Vel);

    }

    public void stopMotors(){

        leftMaster.set(0.0);
        rightMaster.set(0.0);

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

    public boolean getBottomHS(){

        return bottomHardstop.get();

    }

    public boolean getTopHS(){

        return topHardstop.get();

    }

}