package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logic 
{
    private Hardware hardware_;

    private Joystick operaterjoy_ = new Joystick(1);
    private Joystick drivejoy_ = new Joystick(0);

    public void init(Hardware h)
    {
        hardware_ = h;
    }
   
    public void run()
    {
        double speed = drivejoy_.getRawAxis(Hardware.LEFT_STICK_Y);
        double rotate = drivejoy_.getRawAxis(Hardware.RIGHT_STICK_X);
        hardware_.drive(speed * Math.abs(speed), rotate * Math.abs(rotate));
        SmartDashboard.putNumber("sanitycheck", 69420);
        hardware_.checkColour();
    }
    

}
