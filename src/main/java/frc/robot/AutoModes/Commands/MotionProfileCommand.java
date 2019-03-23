/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.AutoModes.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Drive.Drive;

public class MotionProfileCommand extends Command {

  Drive mDrive;
  Robot mRobot;
  String pathName;

  public MotionProfileCommand(String path) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(mDrive);
    pathName = path;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    mDrive = new Drive();
    mDrive.initDefaultCommand();
    mRobot.loadPath(pathName);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    mRobot.followPath();    

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    if(mRobot.m_left_follower.isFinished() || mRobot.m_right_follower.isFinished()){

      return true;      

    }
    else{

      return false;

    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

    mRobot.m_follower_notifier.stop();

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
