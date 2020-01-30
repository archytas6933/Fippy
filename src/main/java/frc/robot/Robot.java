/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot 
{

  public static Hardware hardware_ = new Hardware();
  Logic logic_ = new Logic();

  @Override
  public void robotInit() 
  {
    hardware_.init();
    logic_.init();
  }

  @Override
  public void autonomousInit() 
  {
    logic_.startauto();
  }

  @Override
  public void autonomousPeriodic() 
  {
  }

  @Override
  public void teleopInit() 
  {
    logic_.startteleop();
  }

  @Override
  public void teleopPeriodic() 
  {
    logic_.run();
  }

  @Override
  public void testInit() { }

  @Override
  public void testPeriodic() { }

  @Override
  public void disabledInit() { }

  @Override
  public void disabledPeriodic() { }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
  }
}
