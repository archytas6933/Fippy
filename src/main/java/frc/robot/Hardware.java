package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Hardware 
{
    public static boolean IS_PNEUMATIC = false;

    // ALL MAGIC NUMBERS
    public static int LEFT_CONTROL_MOTOR = 14;
    public static int LEFT_FOLLOW_MOTOR = 12;
    public static int RIGHT_CONTROL_MOTOR = 11;
    public static int RIGHT_FOLLOW_MOTOR = 13; 
    public static int COLOR_WHEEL_MOTOR = 20;
    private static double COLOR_WHEEL_VELOCITY_P = 0.1;
    private static double COLOR_WHEEL_VELOCITY_I = 0.001;
    private static double COLOR_WHEEL_VELOCITY_D = 0;
    private static double COLOR_WHEEL_VELOCITY_F = 0.085;
    private static double COLOR_WHEEL_POSITION_P = 0.15;
    private static double COLOR_WHEEL_POSITION_I = 0; //0.0015;
    private static double COLOR_WHEEL_POSITION_D = 0.2;
    private static int COLOR_WHEEL_VELOCITY_SLOT = 0;
    private static int COLOR_WHEEL_POSITION_SLOT = 1;
    
    private static double LEFT_MOTOR_POSITION_P = 0.3;
    private static double LEFT_MOTOR_POSITION_I = 0; //0.0015;
    private static double LEFT_MOTOR_POSITION_D = 0.2;
    private static int LEFT_MOTOR_POSITION_SLOT = 1;
    
    private static double RIGHT_MOTOR_POSITION_P = 0.3;
    private static double RIGHT_MOTOR_POSITION_I = 0; //0.0015;
    private static double RIGHT_MOTOR_POSITION_D = 0.2;
    private static int RIGHT_MOTOR_POSITION_SLOT = 1;
   
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    public static int PNEUMATICS_ID = 0;
    public static int TESTSOLENOID_ID = 3; 

    
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
    public WPI_VictorSPX leftFollow_;
    public WPI_TalonSRX rightDrive_;
    public WPI_VictorSPX rightFollow_;
    public WPI_TalonSRX colorWheel_;

    public Solenoid testSolenoid_; 

    private UsbCamera camera_;

    private final ColorSensorV3 colorSensor_  = new ColorSensorV3(i2cPort);

    
    // hardware initialization

    public void init()
    {
        leftDrive_ = new WPI_TalonSRX(LEFT_CONTROL_MOTOR);
        leftDrive_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftDrive_.setSensorPhase(true);
        leftDrive_.config_kP(RIGHT_MOTOR_POSITION_SLOT, RIGHT_MOTOR_POSITION_P);
        leftDrive_.config_kI(RIGHT_MOTOR_POSITION_SLOT, RIGHT_MOTOR_POSITION_I);
        leftDrive_.config_kD(RIGHT_MOTOR_POSITION_SLOT, RIGHT_MOTOR_POSITION_D);
        leftFollow_ = new WPI_VictorSPX(LEFT_FOLLOW_MOTOR);
        leftFollow_.follow(leftDrive_);

        rightDrive_ = new WPI_TalonSRX(RIGHT_CONTROL_MOTOR);
        rightDrive_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightDrive_.setSensorPhase(true);
        rightDrive_.config_kP(LEFT_MOTOR_POSITION_SLOT, LEFT_MOTOR_POSITION_P);
        rightDrive_.config_kI(LEFT_MOTOR_POSITION_SLOT, LEFT_MOTOR_POSITION_I);
        rightDrive_.config_kD(LEFT_MOTOR_POSITION_SLOT, LEFT_MOTOR_POSITION_D);
        rightFollow_ = new WPI_VictorSPX(RIGHT_FOLLOW_MOTOR);
        rightFollow_.follow(rightDrive_);

        colorWheel_ = new WPI_TalonSRX(COLOR_WHEEL_MOTOR); 
        colorWheel_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        colorWheel_.config_kP(COLOR_WHEEL_VELOCITY_SLOT, COLOR_WHEEL_VELOCITY_P);
        colorWheel_.config_kI(COLOR_WHEEL_VELOCITY_SLOT, COLOR_WHEEL_VELOCITY_I);
        colorWheel_.config_kD(COLOR_WHEEL_VELOCITY_SLOT, COLOR_WHEEL_VELOCITY_D);
        colorWheel_.config_kF(COLOR_WHEEL_VELOCITY_SLOT, COLOR_WHEEL_VELOCITY_F);
        colorWheel_.config_kP(COLOR_WHEEL_POSITION_SLOT, COLOR_WHEEL_POSITION_P);
        colorWheel_.config_kI(COLOR_WHEEL_POSITION_SLOT, COLOR_WHEEL_POSITION_I);
        colorWheel_.config_kD(COLOR_WHEEL_POSITION_SLOT, COLOR_WHEEL_POSITION_D);

        if (IS_PNEUMATIC)
            testSolenoid_ = new Solenoid(PNEUMATICS_ID, TESTSOLENOID_ID);

        camera_ = CameraServer.getInstance().startAutomaticCapture(0);

        
        
    }

    // hardware helper functions

    public void drive(double speed, double turnrate)
    {
        leftDrive_.set(ControlMode.PercentOutput, speed + turnrate);
        rightDrive_.set(ControlMode.PercentOutput, -speed + turnrate);        
    }

    public void driveLock(double distance)
    {
        leftDrive_.setSelectedSensorPosition(0);
        leftDrive_.set(ControlMode.Position, distance);
        rightDrive_.setSelectedSensorPosition(0);
        rightDrive_.set(ControlMode.Position, -distance);


    }

    public void spinwheel(double speed)
    {
        if (speed == 0) {
            colorWheel_.selectProfileSlot(1, 0);
            colorWheel_.setSelectedSensorPosition(0);
            colorWheel_.set(ControlMode.Position, 0);
            // colorWheel_.set(ControlMode.PercentOutput, 0);
        }
        else{
            colorWheel_.set(ControlMode.PercentOutput, speed);
            // colorWheel_.selectProfileSlot(0, 0);
            // colorWheel_.set(ControlMode.Velocity, speed);
            // intakeposition_ = colorWheel_.getSelectedSensorPosition();
        }
    } 

    public int leftpos()
    {
        return leftDrive_.getSelectedSensorPosition();
    }

    public int rightpos()
    {
        return rightDrive_.getSelectedSensorPosition();
    }

    public void testsol(Boolean isOpen)
    {
        if (!IS_PNEUMATIC)
            return;
        testSolenoid_.set(isOpen);
    }

    public int findColor()
    {
        Color detectedColor = colorSensor_.getColor();
        double red = detectedColor.red;
        double green = detectedColor.green;
        double blue = detectedColor.blue;
        
        if (red > green && red > blue)
            return 0;
        if (green > red && green > blue)
        {
            if (green > 0.55) //(Math.abs(blue - red) < 0.1)
                return 1;
            if (red > blue)
                return 3;
            else return 2;
        }
        return -1;
    }

    public void checkColor()
    {
        Color detectedColor = colorSensor_.getColor();
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
    }

}
