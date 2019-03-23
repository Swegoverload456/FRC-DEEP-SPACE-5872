/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.AutoModes.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Parallelogram.Parallelogram;

public class DropHatch extends Command {

  Parallelogram mP;
  boolean pos;

  public DropHatch(boolean Drop) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    pos = Drop;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    mP = new Parallelogram();
    mP.initDefaultCommand();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    mP.dropHatch(pos);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
