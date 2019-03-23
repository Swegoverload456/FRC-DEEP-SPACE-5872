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
import frc.robot.AutoModes.Test;
import frc.robot.Drive.Drive;
import frc.robot.Parallelogram.Parallelogram;
import frc.robot.Vision.Limelight;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import edu.wpi.first.wpilibj.Notifier;
import java.io.IOException;

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

  public double maxVel = 0;

  Command autoCommand;
  SendableChooser autoChooser;

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
  double previousError = 0;

  boolean shifterTrack = true;

  boolean opC = false;

  //Limelight stuff
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tled = table.getEntry("ledMode");
  NetworkTableEntry tCamMode = table.getEntry("stream");
  public double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  double targetPresent = tv.getDouble(0.0);
  public EncoderFollower m_left_follower;
  public EncoderFollower m_right_follower;
  
  public Notifier m_follower_notifier;
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    mDrive = new Drive();
    mParallelogram = new Parallelogram();
    mLimelight = new Limelight();

    tCamMode.setDouble(0);
    tCamMode.setDouble(2);

    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(360, 240);
    autoChooser = new SendableChooser();
    autoChooser.addDefault("Level 1 To Front Cargo: ", new Test());
    autoChooser.addObject("Left Rocket Double Hatch: ", new Test());
    SmartDashboard.putData("Auto", autoChooser);

    c.setClosedLoopControl(true);

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

  public void loadPath(String pathName){

    try{
      Trajectory left_trajectory = PathfinderFRC.getTrajectory(pathName + ".right");
      Trajectory right_trajectory = PathfinderFRC.getTrajectory(pathName + ".left");

      m_left_follower = new EncoderFollower(left_trajectory);
      m_right_follower = new EncoderFollower(right_trajectory);

      m_left_follower.configureEncoder(mDrive.getLeftTicks(), 4096, 4);
      // You must tune the PID values on the following line!
      m_left_follower.configurePIDVA(0.1, 0.0, 0.005, 0, 0);

      m_right_follower.configureEncoder(mDrive.getRightTicks(), 4096, 4);
      // You must tune the PID values on the following line!
      m_right_follower.configurePIDVA(0.05, 0.0, 0.0025, 0, 0);
      
      m_follower_notifier = new Notifier(this::followPath);
      m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
    }
    catch(IOException e){

      e.printStackTrace();

    }

  }

  @Override
  public void autonomousInit() {

    autoCommand = (Command) autoChooser.getSelected();
    mDrive.resetEncoders();
    mDrive.resetHeading();

    if(autoCommand != null){

      autoCommand.start();

    }

  }

  public void followPath(){
    try {

      if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
        m_follower_notifier.stop();
      } else {
        double left_speed = m_left_follower.calculate(mDrive.getLeftTicks());
        double right_speed = m_right_follower.calculate(mDrive.getRightTicks());
        // double heading = mDrive.getHeading();
        // double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
        // double heading_difference = Pathfinder.boundHalfDegrees(desired_heading -
        // heading);
        // double turn = 0.8 * (-1.0/80.0) * heading_difference;
       
        mDrive.leftMaster.set(left_speed);
        mDrive.rightMaster.set(right_speed);
      }
      
    } catch (Exception e) {

      e.printStackTrace();

    }
  }


  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    while(true){

      double slider = flightStick.getRawAxis(3);

      while (slider < 0.5){

        double leftY = -gamepad.getRawAxis(1);
        double rightX = gamepad.getRawAxis(5) * .75;
        double leftTrigger = gamepad.getRawAxis(3);
        double rightTrigger = gamepad.getRawAxis(4);
        double fLeftY = -flightStick.getRawAxis(1);
        boolean aButton = gamepad.getRawButton(2);
        boolean bButton = gamepad.getRawButton(3);
        boolean xButton = gamepad.getRawButton(1);
        boolean yButton = gamepad.getRawButton(4);
        boolean zButton = gamepad.getRawButton(8);
        boolean red1 = buttonBoard.getRawButton(8);
        boolean red2 = buttonBoard.getRawButton(6);
        boolean pink = buttonBoard.getRawButton(1);
        boolean green = buttonBoard.getRawButton(10);
        boolean yellow1 = buttonBoard.getRawButton(5);
        boolean yellow2 = buttonBoard.getRawButton(3);
        boolean blue1 = buttonBoard.getRawButton(9);
        boolean blue2 = buttonBoard.getRawButton(2);

        double kP = 0.0434782609;
        double kD = 0.434782609;
        double error = tx.getDouble(0);
        //double derivative = (error - previousError) / 11.11111111111111;

        //85% of MAX Speed of both gears in ft/s. This was done to account for voltage differences so we can drive straight.
        double joyToVelHigh = 15.555;
        double joyToVelLow = 4.267;

        double targetVel = leftY * joyToVelHigh;
        double targetTicksPer100ms = targetVel * 12 / 10 * 325.9493235;

        if(timer.get() > 11.111111111111){

          previousError = error;
          timer.reset();
          timer.start();

        }

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

        if(!mParallelogram.getBottomHSR()){

          mParallelogram.resetEncoders();

        }

        if(blue1 && clawToggle == 0){

          clawToggle = 1;

        }
        else if(!blue1 && clawToggle == 1){

          mParallelogram.openClaw(true);
          opC = true;
          clawToggle = 2;

        }
        else if(blue1 && clawToggle == 2){

          clawToggle = 3;

        }
        else if(!blue1 && clawToggle == 3){

          mParallelogram.openClaw(false);
          opC = false;
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

        if(zButton){
            
            if(tx.getDouble(0.0) < -1){

              if(mDrive.getShifterState()){

                shifterTrack = true;
                mDrive.setShifterState(true);
                mDrive.setPower((kP * tx.getDouble(0.0)), -((kP * tx.getDouble(0.0))));

              }
              else if(!mDrive.getShifterState()){

                shifterTrack = false;
                mDrive.setShifterState(true);
                mDrive.setPower((kP * tx.getDouble(0.0)), -((kP * tx.getDouble(0.0))));
    
              }

            }
            else if(tx.getDouble(0.0) > 1){

              if(mDrive.getShifterState()){

                shifterTrack = true;
                mDrive.setShifterState(true);
                mDrive.setPower((kP * tx.getDouble(0.0)), -((kP * tx.getDouble(0.0))));

              }
              else if(!mDrive.getShifterState()){

                shifterTrack = false;
                mDrive.setShifterState(true);
                mDrive.setPower((kP * tx.getDouble(0.0)), -((kP * tx.getDouble(0.0))));
    
              }

            }
            else{

              if(shifterTrack){

                mDrive.setShifterState(false);
                mDrive.setPower(0, 0);

              }
              else{

                mDrive.setPower(0, 0);

              }

          }

        }
        else{

            if(rightX > 0.1 || rightX < -0.1){

              mDrive.setShifterState(true);
              mDrive.setPower((leftY + rightX), (leftY - rightX));

            }
            else{
              
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
            
              mDrive.setPower((leftY + rightX), (leftY - rightX));

            }  

        }

        //if(!mParallelogram.getTopHSL() || !mParallelogram.getTopHSR()){

        if(yellow1){

            mParallelogram.setPos(90000);
            
        }
        else if(yellow2){

            mParallelogram.setPos(600000);

        }
        else if(pink){

            mParallelogram.setPos(1100000);

        }
        else if(green){

            mParallelogram.setPos(950000);

        }
        else if(red2){

            mParallelogram.setPos(900000);

        }
        else if(red1){

            mParallelogram.setPos(400000);

        }
        else if(fLeftY > 0.25 || fLeftY < -0.25){

            mParallelogram.setPower((fLeftY * 0.3846153846), (fLeftY * 0.3846153846));

        }
        else{

          /*if(!mParallelogram.getBottomHSR()){

            mParallelogram.setPower(0, 0);

          }
          else{

            mParallelogram.setPower(-0.3, -0.3);

          }*/
          mParallelogram.setPower(0, 0);

        }

        if(aButton && intakeToggle == 0){

          intakeToggle = 1;

        }
        else if(!aButton && intakeToggle == 1){

          mParallelogram.dropRoller(true);
          mParallelogram.setRoller(0.5);
          mParallelogram.setIntake(0.5);
          intakeToggle = 2;

        }
        else if(aButton && intakeToggle == 2){

          intakeToggle = 3;

        }
        else if(!aButton && intakeToggle == 3){

          mParallelogram.dropRoller(false);
          mParallelogram.dropHatch(false);
          mParallelogram.stopRoller();
          mParallelogram.stopIntake();
          intakeToggle = 0;

        }

        if(bButton && outtakeToggle == 0){

          outtakeToggle = 1;

        }
        else if(!bButton && outtakeToggle == 1){

          mParallelogram.pushBall(true);
          mParallelogram.setIntake(-0.25);
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

        SmartDashboard.putNumber("Current Power: ", leftTrigger);
        SmartDashboard.putNumber("Max Vel: ", mParallelogram.getLeftTicksPer100ms());
        SmartDashboard.putNumber("Target Vel: ", targetTicksPer100ms);
        SmartDashboard.putNumber("LeftLift: ", mParallelogram.getLeftTicks());
        SmartDashboard.putNumber("RightLift: ", mParallelogram.getRightTicks());
        SmartDashboard.putNumber("Left Enc: ", (mDrive.getLeftInches()));
        SmartDashboard.putNumber("Right Enc: ", (mDrive.getRightInches()));
        SmartDashboard.putNumber("Vision X: ", tx.getDouble(0.0));
        SmartDashboard.putNumber("Motor Power: ", mParallelogram.getMotorPower());
        SmartDashboard.putNumber("Lift Count: ", liftCount);
        SmartDashboard.putBoolean("Left Bottom: ", mParallelogram.getBottomHSL());
        SmartDashboard.putBoolean("Right Bottom: ", mParallelogram.getBottomHSR());
        SmartDashboard.putNumber("Area: ", ta.getDouble(0));
        SmartDashboard.putNumber("Heading: ", mDrive.getHeading());
        SmartDashboard.putBoolean("Claw: ", opC);        

      }

    }
    
    /*followPath();
    delay(50);
    loadPath("Test");
    mDrive.resetHeading();
    mDrive.resetEncoders();
    delay(250);
    followPath();
    delay(50);*/
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autoCommand != null) {
      autoCommand.cancel();
    }

    m_follower_notifier.stop();
    mDrive.stopMotors();

    timer.start();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
        Scheduler.getInstance().run();

        double leftY = -gamepad.getRawAxis(1);
        double rightX = gamepad.getRawAxis(5) * .75;
        double leftTrigger = gamepad.getRawAxis(3);
        double rightTrigger = gamepad.getRawAxis(4);
        double fLeftY = -flightStick.getRawAxis(1);
        boolean aButton = gamepad.getRawButton(2);
        boolean bButton = gamepad.getRawButton(3);
        boolean xButton = gamepad.getRawButton(1);
        boolean yButton = gamepad.getRawButton(4);
        boolean zButton = gamepad.getRawButton(8);
        boolean red1 = buttonBoard.getRawButton(8);
        boolean red2 = buttonBoard.getRawButton(6);
        boolean pink = buttonBoard.getRawButton(1);
        boolean green = buttonBoard.getRawButton(10);
        boolean yellow1 = buttonBoard.getRawButton(5);
        boolean yellow2 = buttonBoard.getRawButton(3);
        boolean blue1 = buttonBoard.getRawButton(9);
        boolean blue2 = buttonBoard.getRawButton(2);

        double kP = 0.0434782609;
        double kD = 0.434782609;
        double error = tx.getDouble(0);
        //double derivative = (error - previousError) / 11.11111111111111;

        //85% of MAX Speed of both gears in ft/s. This was done to account for voltage differences so we can drive straight.
        double joyToVelHigh = 15.555;
        double joyToVelLow = 4.267;

        double targetVel = leftY * joyToVelHigh;
        double targetTicksPer100ms = targetVel * 12 / 10 * 325.9493235;

        if(timer.get() > 11.111111111111){

          previousError = error;
          timer.reset();
          timer.start();

        }

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

        if(!mParallelogram.getBottomHSR()){

          mParallelogram.resetEncoders();

        }

        if(blue1 && clawToggle == 0){

          clawToggle = 1;

        }
        else if(!blue1 && clawToggle == 1){

          mParallelogram.openClaw(true);
          opC = true;
          clawToggle = 2;

        }
        else if(blue1 && clawToggle == 2){

          clawToggle = 3;

        }
        else if(!blue1 && clawToggle == 3){

          mParallelogram.openClaw(false);
          opC = false;
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

        if(zButton){
            
            if(tx.getDouble(0.0) < -1){

              if(mDrive.getShifterState()){

                shifterTrack = true;
                mDrive.setShifterState(true);
                mDrive.setPower((kP * tx.getDouble(0.0)), -((kP * tx.getDouble(0.0))));

              }
              else if(!mDrive.getShifterState()){

                shifterTrack = false;
                mDrive.setShifterState(true);
                mDrive.setPower((kP * tx.getDouble(0.0)), -((kP * tx.getDouble(0.0))));
    
              }

            }
            else if(tx.getDouble(0.0) > 1){

              if(mDrive.getShifterState()){

                shifterTrack = true;
                mDrive.setShifterState(true);
                mDrive.setPower((kP * tx.getDouble(0.0)), -((kP * tx.getDouble(0.0))));

              }
              else if(!mDrive.getShifterState()){

                shifterTrack = false;
                mDrive.setShifterState(true);
                mDrive.setPower((kP * tx.getDouble(0.0)), -((kP * tx.getDouble(0.0))));
    
              }

            }
            else{

              if(shifterTrack){

                mDrive.setShifterState(false);
                mDrive.setPower(0, 0);

              }
              else{

                mDrive.setPower(0, 0);

              }

          }

        }
        else{

            if(rightX > 0.1 || rightX < -0.1){

              mDrive.setShifterState(true);
              mDrive.setPower((leftY + rightX), (leftY - rightX));

            }
            else{
              
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
            
              mDrive.setPower((leftY + rightX), (leftY - rightX));

            }  

        }

        //if(!mParallelogram.getTopHSL() || !mParallelogram.getTopHSR()){

        if(yellow1){

            mParallelogram.setPos(90000);
            
        }
        else if(yellow2){

            mParallelogram.setPos(600000);

        }
        else if(pink){

            mParallelogram.setPos(1100000);

        }
        else if(green){

            mParallelogram.setPos(950000);

        }
        else if(red2){

            mParallelogram.setPos(900000);

        }
        else if(red1){

            mParallelogram.setPos(400000);

        }
        else if(fLeftY > 0.25 || fLeftY < -0.25){

            mParallelogram.setPower((fLeftY * 0.3846153846), (fLeftY * 0.3846153846));

        }
        else{

          /*if(!mParallelogram.getBottomHSR()){

            mParallelogram.setPower(0, 0);

          }
          else{

            mParallelogram.setPower(-0.3, -0.3);

          }*/
          mParallelogram.setPower(0, 0);

        }

        if(aButton && intakeToggle == 0){

          intakeToggle = 1;

        }
        else if(!aButton && intakeToggle == 1){

          mParallelogram.dropRoller(true);
          mParallelogram.setRoller(0.5);
          mParallelogram.setIntake(0.5);
          intakeToggle = 2;

        }
        else if(aButton && intakeToggle == 2){

          intakeToggle = 3;

        }
        else if(!aButton && intakeToggle == 3){

          mParallelogram.dropRoller(false);
          mParallelogram.dropHatch(false);
          mParallelogram.stopRoller();
          mParallelogram.stopIntake();
          intakeToggle = 0;

        }

        if(bButton && outtakeToggle == 0){

          outtakeToggle = 1;

        }
        else if(!bButton && outtakeToggle == 1){

          mParallelogram.pushBall(true);
          mParallelogram.setIntake(-0.25);
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

        SmartDashboard.putNumber("Current Power: ", leftTrigger);
        SmartDashboard.putNumber("Max Vel: ", mParallelogram.getLeftTicksPer100ms());
        SmartDashboard.putNumber("Target Vel: ", targetTicksPer100ms);
        SmartDashboard.putNumber("LeftLift: ", mParallelogram.getLeftTicks());
        SmartDashboard.putNumber("RightLift: ", mParallelogram.getRightTicks());
        SmartDashboard.putNumber("Left Enc: ", (mDrive.getLeftInches()));
        SmartDashboard.putNumber("Right Enc: ", (mDrive.getRightInches()));
        SmartDashboard.putNumber("Vision X: ", tx.getDouble(0.0));
        SmartDashboard.putNumber("Motor Power: ", mParallelogram.getMotorPower());
        SmartDashboard.putNumber("Lift Count: ", liftCount);
        SmartDashboard.putBoolean("Left Bottom: ", mParallelogram.getBottomHSL());
        SmartDashboard.putBoolean("Right Bottom: ", mParallelogram.getBottomHSR());
        SmartDashboard.putNumber("Area: ", ta.getDouble(0));
        SmartDashboard.putNumber("Heading: ", mDrive.getHeading());
        SmartDashboard.putBoolean("Claw: ", opC);
    
  } 

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic(){

  }

  public void delay(int ms){

    try{
    		Thread.sleep(ms);
    	}
    	catch(Exception e1){
    		e1.printStackTrace();
    	}

  }

}

