/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandDrive extends CommandBase {
  /**
   * Creates a new CommandDrive.
   */

  private double feet_;
  private DriveSubsystem drive_ = Robot.hardware_.robotdrive_;

  public CommandDrive(double feet) 
  {
    feet_ = feet;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive_.drivelock(feet_);
    
    //
    //System.out.printf("left: %f, right: %f\n", drive_.getLeftDistance(), drive_.getRightDistance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive_.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //double current = drive_.getAverageEncoderDistance();
    double left = drive_.getLeftDistance();
    double right = drive_.getRightDistance();
    //
    System.out.printf("left: %f, right: %f, goal: %f\n", left, right, feet_);
    // return Math.abs(current) > Math.abs(feet_);
    return Math.abs(left) > Math.abs(feet_) && 
      Math.abs(right) > Math.abs(feet_);
  }
}
