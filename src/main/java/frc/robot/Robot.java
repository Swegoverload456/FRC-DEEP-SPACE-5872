/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drive.Drive;
import frc.robot.Leds.Led;
import frc.robot.Parallelogram.Parallelogram;
import frc.robot.Vision.Limelight;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public Drive mDrive;
  public Parallelogram mParallelogram;
  public Limelight mLimelight;
  //public Led mLed;

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

  public double maxVel = 0;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  Timer timer = new Timer();

  Joystick gamepad = new Joystick(0);
  Joystick buttonBoard = new Joystick(1);
  Joystick flightStick = new Joystick(2);
  Compressor c = new Compressor(0);

  int intakeToggle = 0;
  int outtakeToggle = 0;
  int clawToggle = 0;
  int hatchToggle = 0;
  int liftToggle = 0;
  int driveToggle = 0;
  int bottomHatch = 0;
  double liftCount = 0;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tled = table.getEntry("ledMode");

  //read values periodically
  public double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  double targetPresent = tv.getDouble(0.0);
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    mDrive = new Drive();
    mParallelogram = new Parallelogram();
    mLimelight = new Limelight();
    //mLed = new Led();
    mDrive.initDefaultCommand();

    c.setClosedLoopControl(true);
    //mParallelogram.initDefaultCommand();
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

  }

  /**
   * 
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {

    //mLed.set(gold);

  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

   // mLed.set(green);

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    /*timer.start();
    while(((mDrive.getLeftVelInchesPerSec() + mDrive.getRightVelInchesPerSec()/2)) < 109.8){

      mDrive.setVel(109.8, 109.8);
      System.out.println("Current Vel: " + (mDrive.getLeftVelInchesPerSec() + mDrive.getRightVelInchesPerSec())/2 + "Current Accel: " + ((mDrive.getLeftVelInchesPerSec() + mDrive.getRightVelInchesPerSec())/2/timer.get()) + "Current Time: " + timer.get());

    }*/

    mParallelogram.setMotionMagic(50000);

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
        Scheduler.getInstance().run();

        double leftY = -gamepad.getRawAxis(1);
        double rightX = gamepad.getRawAxis(4) * .75;
        double leftTrigger = gamepad.getRawAxis(2);
        double rightTrigger = gamepad.getRawAxis(3);
        double fLeftY = -flightStick.getRawAxis(1);
        boolean aButton = gamepad.getRawButton(1);
        boolean bButton = gamepad.getRawButton(2);
        boolean xButton = gamepad.getRawButton(3);
        boolean yButton = gamepad.getRawButton(4);
        boolean leftBumper = gamepad.getRawButton(5);
        boolean red1 = buttonBoard.getRawButton(8);
        boolean red2 = buttonBoard.getRawButton(6);
        boolean pink = buttonBoard.getRawButton(1);
        boolean green = buttonBoard.getRawButton(10);
        boolean yellow1 = buttonBoard.getRawButton(5);
        boolean yellow2 = buttonBoard.getRawButton(3);
        boolean blue1 = buttonBoard.getRawButton(9);
        boolean blue2 = buttonBoard.getRawButton(2);

        double kP = 0.0434782609;

        //85% of MAX Speed of both gears in ft/s. This was done to account for voltage differences so we can drive straight.
        double joyToVelHigh = 15.555;
        double joyToVelLow = 4.267;

        double targetVel = leftY * joyToVelHigh;
        double targetTicksPer100ms = targetVel * 12 / 10 * 325.9493235;

        if(xButton && driveToggle == 0){

        driveToggle = 1;

        }
        else if(!xButton && driveToggle == 1){

        mDrive.setShifterState(true);
        driveToggle = 2;

        }
        else if(xButton && driveToggle == 2){

        driveToggle = 3;

        }
        else if(!xButton && driveToggle == 3){

        mDrive.setShifterState(false);
        driveToggle = 0;

        }

        if(yButton && liftToggle == 0){

        liftToggle = 1;

        }
        else if(!yButton && liftToggle == 1){

        mDrive.engageLift(true);
        mDrive.releaseLift(true);
        liftToggle = 2;

        }
        else if(yButton && liftToggle == 2){

        liftToggle = 3;

        }
        else if(!yButton && liftToggle == 3){

        mDrive.engageLift(false);
        mDrive.releaseLift(false);
        liftToggle = 0;

        }

        if(mParallelogram.getBottomHSL() && mParallelogram.getBottomHSR()){

        mParallelogram.resetEncoders();

        }

        if(blue1 && clawToggle == 0){

        clawToggle = 1;

        }
        else if(!blue1 && clawToggle == 1){

        mParallelogram.openClaw(true);
        clawToggle = 2;

        }
        else if(blue1 && clawToggle == 2){

        clawToggle = 3;

        }
        else if(!blue1 && clawToggle == 3){

        mParallelogram.openClaw(false);
        clawToggle = 0;

        }

        if(blue2 && hatchToggle == 0){

        hatchToggle = 1;

        }
        else if(!blue2 && hatchToggle == 1){

        mParallelogram.dropRoller(true);
        mParallelogram.dropHatch(true);
        hatchToggle = 2;

        }
        else if(blue2 && hatchToggle == 2){

        hatchToggle = 3;

        }
        else if(!blue2 && hatchToggle == 3){

        mParallelogram.dropHatch(false);
        mParallelogram.dropRoller(false);
        hatchToggle = 0;

        }

        if(leftBumper){

        if(tx.getDouble(0.0) < -1){

            mDrive.setPower((kP * tx.getDouble(0.0)), -(kP * tx.getDouble(0.0)));

        }
        else if(tx.getDouble(0.0) > 1){

            mDrive.setPower((kP * tx.getDouble(0.0)), -(kP * tx.getDouble(0.0)));

        }
        else{

            mDrive.setPower(0, 0);

        }

        }
        else{

            if(leftY < 0.02){

            mDrive.setPower((leftY + rightX), (leftY - rightX));

            }
            else{

            mDrive.setPower((leftY * 0.4 + rightX), (leftY * 0.4 - rightX));

            }  

        }

        if(!mParallelogram.getTopHSL() || !mParallelogram.getTopHSR()){

        if(yellow1){

            mParallelogram.setPos(90000);
            
        }
        else if(yellow2){

            mParallelogram.setPos(675000);

        }
        else if(pink){

            mParallelogram.setPos(900000);

        }
        else if(green){

            mParallelogram.setPos(920000);

        }
        else if(red2){

            mParallelogram.setPos(700000);

        }
        else if(red1){

            mParallelogram.setPos(110000);

        }
        else if(fLeftY > 0.25 || fLeftY < -0.25){

            mParallelogram.setPower((fLeftY * 0.3846153846), (fLeftY * 0.3846153846));

        }
        else{

            if(!mParallelogram.getBottomHSL()){

            mParallelogram.setLeftPower(-0.25);

            }
            else{

            mParallelogram.setLeftPower(0);

            }

            if(!mParallelogram.getBottomHSR()){

            mParallelogram.setRightPower(-0.25);

            }
            else{

            mParallelogram.setRightPower(0);

            }

        }

        if(aButton && intakeToggle == 0){

        intakeToggle = 1;

        }
        else if(!aButton && intakeToggle == 1){

        mParallelogram.dropRoller(true);
        mParallelogram.setRoller(1.0);
        mParallelogram.setIntake(1.0);
        intakeToggle = 2;

        }
        else if(aButton && intakeToggle == 2){

        intakeToggle = 3;

        }
        else if(!aButton && intakeToggle == 3){

        mParallelogram.dropRoller(false);
        mParallelogram.stopRoller();
        mParallelogram.stopIntake();
        intakeToggle = 0;

        }

        if(bButton && outtakeToggle == 0){

        outtakeToggle = 1;

        }
        else if(!bButton && outtakeToggle == 1){

        mParallelogram.pushBall(true);
        mParallelogram.setIntake(-0.75);

        outtakeToggle = 2;

        }
        else if(bButton && outtakeToggle == 2){

        outtakeToggle = 3;

        }
        else if(!bButton && outtakeToggle == 3){

        mParallelogram.pushBall(false);
        mParallelogram.stopIntake();
        outtakeToggle = 0;

        }

        if((mParallelogram.getLeftTicksPer100ms() + mParallelogram.getRightTicksPer100ms()) / 2 < maxVel){

        maxVel = (mParallelogram.getLeftTicksPer100ms() + mParallelogram.getRightTicksPer100ms()) / 2;

        }

        //mDrive.setVel(targetTicksPer100ms, targetTicksPer100ms);
        //mDrive.setVel((leftY + rightX * joyToVelHigh), (leftY - rightX * joyToVelHigh));
        SmartDashboard.putNumber("Current Power: ", leftTrigger);
        SmartDashboard.putNumber("Max Vel: ", mParallelogram.getLeftTicksPer100ms());
        SmartDashboard.putNumber("Target Vel: ", targetTicksPer100ms);
        SmartDashboard.putNumber("LeftLift: ", mParallelogram.getLeftTicks());
        SmartDashboard.putNumber("RightLift: ", mParallelogram.getRightTicks());
        SmartDashboard.putNumber("Left Enc: ", (mDrive.getLeftVelInchesPerSec()));
        SmartDashboard.putNumber("Right Enc: ", (mDrive.getRightVelInchesPerSec()));
        SmartDashboard.putNumber("Vision X: ", tx.getDouble(0.0));
        SmartDashboard.putNumber("Motor Power: ", mParallelogram.getMotorPower());
        SmartDashboard.putNumber("Lift Count: ", liftCount);

    }

  } 

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic(){

    //Scheduler.getInstance.run();
    //mLimelight.ledOff();

  }

 /* public void autoTurn(){

    double threshold = 0.5;
    while(mLimelight.getXPos() > threshold || mLimelight.getXPos() < -threshold){

      mDrive.turn(mLimelight.getXPos());
      if(mLimelight.getXPos() < threshold && mLimelight.getXPos() > -threshold){

        mLed.blink(gold, 250, 5);

      }
      else{

        mLed.set(red);

      }
      break;

    }

  }*/
}

