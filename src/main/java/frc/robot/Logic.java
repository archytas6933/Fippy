package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Logic 
{
    private Hardware hardware_;

    private Joystick operatorjoy_ = new Joystick(1);
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

        boolean solenoid = operatorjoy_.getRawButton(Hardware.ABUTTON);
        hardware_.testsol(solenoid);
    }

}
