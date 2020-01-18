package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

public class Hardware 
{
    // ALL MAGIC NUMBERS
    public static int LEFT_FRONT_MOTOR = 12;
    public static int LEFT_BACK_MOTOR = 14;
    public static int RIGHT_FRONT_MOTOR = 11;
    public static int RIGHT_BACK_MOTOR = 13; 

    
    public static int ABUTTON = 1;
	public static int BBUTTON = 2;
	public static int XBUTTON = 3;
	public static int YBUTTON = 4;
	public static int LBBUTTON = 5;
	public static int RBBUTTON = 6;
	public static int BACKBUTTON = 7;
	public static int STARTBUTTON = 8;
	public static int LEFTJOYCLICK = 9;
    public static int RIGHTJOYCLICK = 10;
    
    public static int LEFT_STICK_X = 0;
    public static int LEFT_STICK_Y = 1;
  
    public static int RIGHT_STICK_X = 4;
    public static int RIGHT_STICK_Y = 5;
  


    // hardware object declarations

    public WPI_TalonSRX leftDrive_;
    public WPI_TalonSRX leftFollow_;
    public WPI_TalonSRX rightDrive_;
    public WPI_TalonSRX rightFollow_;

    private UsbCamera camera_;
    
    // hardware initialization

    public void init()
    {
        leftDrive_ = new WPI_TalonSRX(LEFT_FRONT_MOTOR);
        leftFollow_ = new WPI_TalonSRX(LEFT_BACK_MOTOR);
        
        rightDrive_ = new WPI_TalonSRX(RIGHT_FRONT_MOTOR);
        rightFollow_ = new WPI_TalonSRX(RIGHT_BACK_MOTOR);

        camera_ = CameraServer.getInstance().startAutomaticCapture(0);
        
    }

    // hardware helper functions

    public void drive(double speed, double turnrate)
    {
        leftDrive_.set(ControlMode.PercentOutput, -speed + turnrate);
        rightDrive_.set(ControlMode.PercentOutput, speed + turnrate);        
    }



}
