package frc.robot;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Logic 
{
    private static final String[] PATHS = {
        "TEST,D2.5,R180,D2.5",
        "DIRECT,W2,D2.5,S2,D-1,R45,D-4",
        "STEAL,D2.5,S5,D-1,R45,D-4",
        "DUMPCOLLECT,D2.5,S5,D-1,R45,D-4"
    };
    private int currentauto_;
    private Hardware hardware_;
    public double colorRotations_;
    public int previousColor_;
    private String colorHistory_;
//    private boolean isAutoDriving_;
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
        currentauto_ = 0;

        hardware_ = Robot.hardware_;

        SmartDashboard.putBoolean("NEXTAUTO", false);
        SmartDashboard.putString("AUTOPATH", PATHS[0]);
        // isAutoDriving_ = false;
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
        boolean istitle = true;
        for (String step: pathlist)
        {
            if (istitle)
                istitle = false;
            else {
                // System.out.println(step);
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
        }
        auto_.schedule();
    }
    public void startauto()
    {
        String path = SmartDashboard.getString("AUTOPATH", "");
        if (path != "")
            runpath(path);
    }

    public void startteleop()
    {
        // System.out.println("CANCELLING ALL COMMANDS");
        CommandScheduler.getInstance().cancelAll();
        hardware_.brake();
    }
    
    // runs every 20 ms during TELEOP
    public void run()
    {    
        hardware_.climbwithwinch(operatorjoy_.getRawAxis(Hardware.RTAXIS));
        
        if (operatorjoy_.getRawAxis(Hardware.LTAXIS) != 0)
            hardware_.climbwithwinch(-operatorjoy_.getRawAxis(Hardware.LTAXIS));
        
        hardware_.liftsabe(0); 
        
        if(operatorjoy_.getPOV() == Hardware.DAXISN)
            hardware_.liftwheel(-0.8);
        else if(operatorjoy_.getPOV() == Hardware.DAXISS)
            hardware_.liftwheel(0.3);
        else hardware_.liftwheel(0);

        if (operatorjoy_.getRawButton(Hardware.YBUTTON))
            hardware_.liftsabe(1);
        if (operatorjoy_.getRawButton(Hardware.ABUTTON))
            hardware_.liftsabe(-1);

         
        double intakeAxis = operatorjoy_.getRawAxis(Hardware.LEFT_STICK_Y);     
        double deliverAxis = operatorjoy_.getRawAxis(Hardware.RIGHT_STICK_Y);

        SmartDashboard.putNumber("floor", hardware_.floor_.getSelectedSensorPosition());
        // SmartDashboard.putNumber("is limitf closed",
        //     hardware_.floor_.isFwdLimitSwitchClosed());
        // SmartDashboard.putNumber("is limitr closed", 
        //     hardware_.floor_.isRevLimitSwitchClosed());

        if (Math.abs(intakeAxis) > 0.2) {
            hardware_.intake(intakeAxis);
        }
        else {
            hardware_.intake(0);
            if (Math.abs(deliverAxis) > 0.2) {
                if (deliverAxis > 0)
                {
                    //drop
                    hardware_.fipptuate(deliverAxis);
                    hardware_.letThereBeFloor(true);
                }
                else
                {
                    //deliver
                    hardware_.fipptuate(deliverAxis);
                    hardware_.letThereBeFloor(false);
                }
            }
            else
                hardware_.fipptuate(0);        
        }
        
        if(drivejoy_.getPOV() == Hardware.DAXISN)
           isprecisionmode_ = true;
        if (drivejoy_.getPOV() == Hardware.DAXISS)
            isprecisionmode_ = false;
        
        if (operatorjoy_.getRawButton(Hardware.RBBUTTON))
            {
                int color = hardware_.findColor();
                if (color != checkgamecolor() && !isColorFound_)
                    hardware_.spinwheel(-1);
                else 
                    {
                    if (!isColorFound_)
                        hardware_.spinwheel(0);
                    isColorFound_ = true;
                    }    
                if (previousColor_ == -1)
                {
                    previousColor_ = color; 
                    // isAutoDriving_ = true;
                }
                //hardware_.drive(0.2, 0);
                int distance = ((color + 4 - previousColor_) % 4);
                if (distance != 3)
                    {
                    colorRotations_ += distance * 0.125;
                    previousColor_ = color;
                    }
            }
        else
            {
                // if (previousColor_ != -1) 
                    hardware_.spinwheel(0);
                previousColor_ = -1;
                // isAutoDriving_ = false;
                isColorFound_ = false;
            }
        // hardware_.checkColor();
            
        // if (!isAutoDriving_)
        // {

        double speed = drivejoy_.getRawAxis(Hardware.LEFT_STICK_Y);
        double intakespeed =  drivejoy_.getRawAxis(Hardware.LTAXIS);
        double deliveryspeed = drivejoy_.getRawAxis(Hardware.RTAXIS);
        double rotate = drivejoy_.getRawAxis(Hardware.RIGHT_STICK_X);
        speed += intakespeed;
        speed -= deliveryspeed;
        if (isprecisionmode_)
            {
                speed = speed / 2;
                rotate = rotate / 2;
            }

        // if (drivejoy_.getRawButton(Hardware.RBBUTTON))
        // {
        //     if (!isBraking_)
        //         hardware_.brake();
        //     isBraking_ = true;
        // }
        // else
        // {

        hardware_.drive(speed * Math.abs(speed), rotate * Math.abs(rotate));
        //     isBraking_ = false;
        // }
            
        // }
    }

    public void updatedashboard()
    {
        SmartDashboard.putBoolean("isprecision", isprecisionmode_);
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

        if (SmartDashboard.getBoolean("NEXTAUTO", false)) {
            SmartDashboard.putBoolean("NEXTAUTO", false);
            currentauto_++;
            if (currentauto_ >= PATHS.length)
                currentauto_ = 0;
            SmartDashboard.putString("AUTOPATH", PATHS[currentauto_]);
        }


    }

}
