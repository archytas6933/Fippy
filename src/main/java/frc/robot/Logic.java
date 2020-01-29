package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Logic 
{
    private Hardware hardware_;
    public double colorRotations_;
    public int previousColor_;
    private String colorHistory_;
    private boolean isAutoDriving_;
    private String gameTargetColor_;
    private boolean isColorFound_;

    private Joystick drivejoy_ = new Joystick(0);
//    private Joystick operatorjoy_ = new Joystick(1);

    SequentialCommandGroup auto_;

    public void init()
    {
        colorRotations_ = 0;
        previousColor_ = -1;
        colorHistory_ = "";
        isColorFound_ = false;

        hardware_ = Robot.hardware_;

        isAutoDriving_ = false;
    }
    
    public String currentcolor() 
    {
        switch (hardware_.findColor())
        {
            case 0: return "blue";
            case 1: return "yellow";
            case 2: return "red";
            case 3: return "green";
        }
        return "";
    }

    // runs every 20 ms during TELEOP
    public void run()
    {
        gameTargetColor_= DriverStation.getInstance().getGameSpecificMessage();
        checkgamecolor();
        SmartDashboard.putNumber("turns", colorRotations_);
        SmartDashboard.putString("colorHistory", colorHistory_);
        SmartDashboard.putString("color:", currentcolor());
        SmartDashboard.putString("goalcolor", gameTargetColor_);

        // SmartDashboard.putNumber("leftpos", hardware_.leftpos());
        // SmartDashboard.putNumber("rightpos", hardware_.rightpos());
   
        if (drivejoy_.getRawButton(Hardware.ABUTTON))
            {
                int color = hardware_.findColor();
                if (color != checkgamecolor() && !isColorFound_)
                    hardware_.spinwheel(1);
                else 
                    {
                    if (!isColorFound_)
                        hardware_.spinwheel(0);
                    isColorFound_ = true;
                    }    
                if (previousColor_ == -1)
                {
                    previousColor_ = color; 
                    hardware_.driveLock(500);
                    isAutoDriving_ = true;
                }
                int distance = ((color + 4 - previousColor_) % 4);
                if (distance != 3)
                    {
                    colorRotations_ += distance * 0.125;
                    previousColor_ = color;
                    }
            }
        else
            {
                if (previousColor_ != -1) 
                    hardware_.spinwheel(0);
                previousColor_ = -1;
                isAutoDriving_ = false;
                isColorFound_ = false;
            }
        // hardware_.checkColor();
            
        if (!isAutoDriving_)
        {
            double speed = - drivejoy_.getRawAxis(Hardware.LEFT_STICK_Y);
            double rotate = drivejoy_.getRawAxis(Hardware.RIGHT_STICK_X);
            hardware_.drive(speed * Math.abs(speed), rotate * Math.abs(rotate));
        }
    }

    public int checkgamecolor()
    {
        if(gameTargetColor_.length() > 0)
        {
            switch (gameTargetColor_.charAt(0))
            {
                case 'B': return 0;
                case 'G': return 3;
                case 'R': return 2;
                case 'Y': return 1;
            }
        }
        return -1;
    }
    public void startauto()
    {
        auto_ = new SequentialCommandGroup();
        auto_.addCommands(new CommandDrive(0.3));
        auto_.addCommands(new CommandDeliver());
        auto_.addCommands(new CommandDrive(-0.3));
        auto_.schedule();
    }

    // runs every 20 ms during AUTO
    public void runauto()
    {
    }
    

}
