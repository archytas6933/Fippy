/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandTurn extends CommandBase {
  private double degrees_;
  private DriveSubsystem drive_ = Robot.hardware_.robotdrive_;
  /**
   * Creates a new CommandTurn.
   */
  public CommandTurn(double degrees) 
  {
    degrees_ = degrees;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive_.zeroHeading();
    drive_.turnLock(degrees_);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //System.out.printf("current: %f, goal: %f\n", drive_.getHeading(), degrees_);
   return Math.abs(degrees_ - drive_.getHeading()) < 3;
  }
}
