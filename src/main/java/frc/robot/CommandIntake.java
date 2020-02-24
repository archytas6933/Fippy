/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandIntake extends CommandBase {
  /**
   * Creates a new CommandIntake.
   */
  public double feet_;
  public CommandIntake(double feet) {
    feet_ = feet;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (feet_ < 0) 
      Robot.hardware_.intake(-1);
    else {
      Robot.hardware_.intake(1);
      Robot.hardware_.robotdrive_.drivelock(feet_);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.hardware_.intake(0);
    Robot.hardware_.drive(0, 0);
    Robot.hardware_.fipptuate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (feet_ < 0)
      return false;
    
    double left = Robot.hardware_.robotdrive_.getLeftDistance();
    double right = Robot.hardware_.robotdrive_.getRightDistance();
    return Math.abs(left) > Math.abs(feet_) && 
      Math.abs(right) > Math.abs(feet_);
  }
}
