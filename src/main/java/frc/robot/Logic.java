package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Logic 
{
    private static final String PATH_TEST = "D2.5,R180,D2";
    private Hardware hardware_;
    public double colorRotations_;
    public int previousColor_;
    private String colorHistory_;
    private boolean isAutoDriving_;
    private String gameTargetColor_;
    private boolean isColorFound_;
    private boolean isBraking_;
    private boolean isprecisionmode_;

    private Joystick drivejoy_ = new Joystick(0);
    private Joystick operatorjoy_ = new Joystick(1);

    SequentialCommandGroup auto_;

    public void init()
    {
        colorRotations_ = 0;
        previousColor_ = -1;
        colorHistory_ = "";
        isColorFound_ = false;
        isBraking_ = false;
        isprecisionmode_ = false;

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
    public void runpath (String path)
    {
        auto_ = new SequentialCommandGroup();
        String[] pathlist = path.split(",");
        for (String step: pathlist)
        {
            System.out.println(step);
            char ctype = step.charAt(0);
            step = step.substring(1);
            double value = Double.parseDouble(step);
            switch (ctype)
            {
                case 'D': auto_.addCommands(new CommandDrive(value));
                    break;
                case 'R': auto_.addCommands(new CommandTurn(value));
                    break;
                case 'W': auto_.addCommands(new CommandWait().withTimeout(value));
                    break;
                case 'I': auto_.addCommands(new CommandIntake(value));
                    break;
                case 'S': auto_.addCommands(new CommandDeliver().withTimeout(value));
                    break;
            }
        }
        auto_.schedule();
    }
    public void startauto()
    {
       runpath(PATH_TEST);
    }

    public void startteleop()
    {
        System.out.println("CANCELLING ALL COMMANDS");
        CommandScheduler.getInstance().cancelAll();
        hardware_.brake();
    }
    
    // runs every 20 ms during TELEOP
    public void run()
    {
        gameTargetColor_= DriverStation.getInstance().getGameSpecificMessage();
        checkgamecolor();
        SmartDashboard.putNumber("turns", colorRotations_);
        SmartDashboard.putString("color:", currentcolor());
        SmartDashboard.putString("goalcolor", gameTargetColor_);
        double mps = ((-hardware_.robotdrive_.getRightRate()+
            hardware_.robotdrive_.getLeftRate())/2)*10;
        SmartDashboard.putNumber("MPS", mps);
        //display angle on SmartDashboard
        SmartDashboard.putNumber("heading", hardware_.robotdrive_.getHeading());
        SmartDashboard.putNumber("distance", hardware_.robotdrive_.getAverageEncoderDistance());

        // SmartDashboard.putNumber("leftpos", hardware_.leftpos());
        // SmartDashboard.putNumber("rightpos", hardware_.rightpos());
        
    
        hardware_.climbwithwinch(operatorjoy_.getRawAxis(Hardware.RTAXIS));
        
        if (operatorjoy_.getRawAxis(Hardware.LTAXIS) != 0)
            hardware_.climbwithwinch(-operatorjoy_.getRawAxis(Hardware.LTAXIS));
        
        hardware_.liftsabe(0); 
        if (operatorjoy_.getRawButton(Hardware.YBUTTON))
            hardware_.liftsabe(1);
        if (operatorjoy_.getRawButton(Hardware.ABUTTON))
            hardware_.liftsabe(-1);

         
        double intakeAxis = operatorjoy_.getRawAxis(Hardware.LEFT_STICK_Y);     
        double deliverAxis = operatorjoy_.getRawAxis(Hardware.RIGHT_STICK_Y);
        
        if (intakeAxis > 0)
        {
            //drop
            hardware_.intake(intakeAxis);
            hardware_.fipptuate(intakeAxis);
            hardware_.letThereBeFloor(1);
        }
        else
        {
            //intake 
            hardware_.letThereBeFloor(-1);
            hardware_.fipptuate(intakeAxis);
            hardware_.intake(intakeAxis);
        }   
        if (deliverAxis > 0)
        {
            //drop
            hardware_.fipptuate(deliverAxis);
            hardware_.letThereBeFloor(1);
        }
        else
        {
            //deliver
            hardware_.letThereBeFloor(-1);
            hardware_.fipptuate(deliverAxis);
        }
    
        
        if(drivejoy_.getPOV() == Hardware.DAXISN)
           isprecisionmode_ = true;
        if (drivejoy_.getPOV() == Hardware.DAXISS)
            isprecisionmode_ = false;
        SmartDashboard.putBoolean("isprecision", isprecisionmode_);
        
        if (drivejoy_.getRawButton(Hardware.LBBUTTON))
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
                    isAutoDriving_ = true;
                }
                hardware_.drive(0.2, 0);
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
            double intakespeed =  drivejoy_.getRawAxis(Hardware.RTAXIS);
            double deliveryspeed = drivejoy_.getRawAxis(Hardware.LTAXIS);
            double rotate = drivejoy_.getRawAxis(Hardware.RIGHT_STICK_X);
            speed += intakespeed;
            speed -= deliveryspeed;
            if (isprecisionmode_)
                {
                    speed = speed / 2;
                    rotate = rotate / 2;
                }
            if (drivejoy_.getRawButton(Hardware.RBBUTTON))
            {
                if (!isBraking_)
                    hardware_.brake();
                isBraking_ = true;
            }
            else
            {

                hardware_.drive(speed * Math.abs(speed), rotate * Math.abs(rotate));
                
                isBraking_ = false;
            }
            
        }
    }

}
