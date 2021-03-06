/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandDeliver extends CommandBase {
  /**
   * Creates a new CommandDeliver.
   */
  public CommandDeliver() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.hardware_.drive(-0.15, 0);
    Robot.hardware_.fipptuate(1);
    Robot.hardware_.letThereBeFloor(Hardware.FLOOR_DELIVER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.hardware_.fipptuate(0);
    Robot.hardware_.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
