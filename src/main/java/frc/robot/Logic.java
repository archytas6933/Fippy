package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logic 
{
    private Hardware hardware_;

    private Joystick drivejoy_ = new Joystick(0);
    private Joystick operatorjoy_ = new Joystick(1);

    public void init(Hardware h)
    {
        hardware_ = h;
    }
   
    // runs every 20 ms during TELEOP
    public void run()
    {

        double speed = drivejoy_.getRawAxis(Hardware.LEFT_STICK_Y);
        double rotate = drivejoy_.getRawAxis(Hardware.RIGHT_STICK_X);
        hardware_.drive(speed * Math.abs(speed), rotate * Math.abs(rotate));

        hardware_.testsol(operatorjoy_.getRawButton(Hardware.ABUTTON));

        hardware_.checkColor();
    }

    public void startauto()
    {

    }

    // runs every 20 ms during AUTO
    public void runauto()
    {

    }
    

}
