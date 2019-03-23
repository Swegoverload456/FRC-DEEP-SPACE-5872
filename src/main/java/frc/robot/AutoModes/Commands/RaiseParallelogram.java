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
import frc.robot.Parallelogram.Parallelogram;

public class RaiseParallelogram extends Command {

  Parallelogram mParallelogram = new Parallelogram();
  double h;
  double threshold = 5000;

  public RaiseParallelogram(double height) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    h = height;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    mParallelogram.initDefaultCommand();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    mParallelogram.setPos(h);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(mParallelogram.getLeftTicks() + threshold < h || mParallelogram.getLeftTicks() - threshold > h){

      return false;

    }
    else{

      return true;

    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
