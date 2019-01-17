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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  public static OI mOi;

  public Drive mDrive;
  public Parallelogram mParallelogram;
  public Limelight mLimelight;
  public Led mLed;

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

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  Timer timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    mOi = new OI();
    mDrive.initDefaultCommand();
    mParallelogram.initDefaultCommand();
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

    mLed.set(gold);

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

    mLed.set(green);

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

    timer.start();
    while(((mDrive.getLeftVelInchesPerSec() + mDrive.getRightVelInchesPerSec()/2)) < 109.8){

      mDrive.setVel(109.8, 109.8);
      System.out.println("Current Vel: " + (mDrive.getLeftVelInchesPerSec() + mDrive.getRightVelInchesPerSec())/2 + "Current Accel: " + ((mDrive.getLeftVelInchesPerSec() + mDrive.getRightVelInchesPerSec())/2/timer.get()) + "Current Time: " + timer.get());

    }

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

    double cLeftYAxis = mOi.controller.getRawAxis(0);
    double cRightXAxis = mOi.controller.getRawAxis(3);

    if(cLeftYAxis > 0.03 || cLeftYAxis < 0.03 || cRightXAxis > 0.03 || cRightXAxis < 0.03){

      if(mDrive.getShifterState()){

        mDrive.setVel((((cLeftYAxis + cRightXAxis) / 2) * Constants.kHighGearVelC), (((cLeftYAxis - cRightXAxis) / 2) * Constants.kHighGearVelC));

      }
      else{

        mDrive.setVel((((cLeftYAxis + cRightXAxis) / 2) * Constants.kLowGearVelC), (((cLeftYAxis - cRightXAxis) / 2) * Constants.kLowGearVelC));

      }

    }
    else{

      mDrive.stopMotors();

    }

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

  public void autoTurn(){

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

  }

}
