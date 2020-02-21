package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Hardware 
{
    public static boolean IS_PNEUMATIC = false;

    // ALL MAGIC NUMBERS

    public static final double ksVolts = 0.918;
    public static final double kvVoltSecondsPerMeter = 3.0;
    public static final double kaVoltSecondsSquaredPerMeter = 0.542;

    public static final double kPDriveVel = 39.4;
    public static final double kTrackwidthMeters = 0.588;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
        
    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 10;            

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    
    public static int WINCH_MOTOR = 20;
    public static int LET_THERE_BE_FLOOR = 10;
    public static int COLOR_WHEEL_MOTOR = 19;

    private static int POSITION_SLOT = 0;
    private static int VELOCITY_SLOT = 1;

    private static double FLOOR_POSITION_P = 0.4;
    private static double FLOOR_POSITION_I = 0; //0.0015;
    private static double FLOOR_POSITION_D = 0.1;

    public static int FLOOR_TOP = 0;
    public static int FLOOR_BOTTOM = -1300;
    public static int FLOOR_DELIVER = -200;
    public static int FLOOR_RELAY = -600;

    public static int INTAKE_MOTOR = 18;
    private static double INTAKE_MOTOR_VELOCITY_P = 0.1;
    private static double INTAKE_MOTOR_VELOCITY_I = 0.001;
    private static double INTAKE_MOTOR_VELOCITY_D = 0;
    private static double INTAKE_MOTOR_VELOCITY_F = 0.085;    

    public static int FIPPY_ROLLER_MOTOR = 17;
    private static double FIPPY_ROLLER_MOTOR_VELOCITY_P = 0.15;
    private static double FIPPY_ROLLER_MOTOR_VELOCITY_I = 0.001;
    private static double FIPPY_ROLLER_MOTOR_VELOCITY_D = 0;
    private static double FIPPY_ROLLER_MOTOR_VELOCITY_F = 0.085;
        
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
    
	public static final int DAXISN = 0;
    public static final int DAXISS = 180;

	public static final int RTAXIS = 3;

	public static final int LTAXIS = 2;  
    
    public static int LEFT_STICK_X = 0;
    public static int LEFT_STICK_Y = 1;
  
    public static int RIGHT_STICK_X = 4;
    public static int RIGHT_STICK_Y = 5;
  
    public static double INTAKE_FIPP = 0.7;


    // hardware object declarations

    public WPI_TalonSRX colorWheel_;
    public WPI_TalonSRX climbWinch_;
    public WPI_TalonSRX intakeMotor_;
    public WPI_TalonSRX fippyRoller_;
    public WPI_TalonSRX floor_;

    public Spark lightSaber_;
    public Spark wheelPosition_;

    public Solenoid testSolenoid_; 

    private UsbCamera camera_;
    private UsbCamera camera2_;

    private final ColorSensorV3 colorSensor_  = new ColorSensorV3(i2cPort);

    public DriveSubsystem robotdrive_ = new DriveSubsystem();

    // hardware initialization

    public void init()
    {
        colorWheel_ = new WPI_TalonSRX(COLOR_WHEEL_MOTOR); 
        colorWheel_.setNeutralMode(NeutralMode.Coast);

        intakeMotor_ = new WPI_TalonSRX(INTAKE_MOTOR);
        fippyRoller_ = new WPI_TalonSRX(FIPPY_ROLLER_MOTOR);
        climbWinch_ = new WPI_TalonSRX(WINCH_MOTOR);
        colorWheel_ = new WPI_TalonSRX(COLOR_WHEEL_MOTOR);
        climbWinch_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        lightSaber_ = new Spark(0);
        wheelPosition_ = new Spark(1);

        if (IS_PNEUMATIC)
            testSolenoid_ = new Solenoid(PNEUMATICS_ID, TESTSOLENOID_ID);

        camera_ = CameraServer.getInstance().startAutomaticCapture(0);
        camera2_= CameraServer.getInstance().startAutomaticCapture(1);
        camera2_.setResolution(120, 120);
        camera_.setResolution(120, 120);
        
        floor_= new WPI_TalonSRX(LET_THERE_BE_FLOOR);
        floor_.configFactoryDefault();
        floor_.configForwardLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector, 
            LimitSwitchNormal.NormallyClosed);
        floor_.configReverseLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector, 
            LimitSwitchNormal.NormallyClosed);
        floor_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        floor_.config_kP(POSITION_SLOT, FLOOR_POSITION_P);
        floor_.config_kI(POSITION_SLOT, FLOOR_POSITION_I);
        floor_.config_kD(POSITION_SLOT, FLOOR_POSITION_D);
        floor_.setSelectedSensorPosition(0);
    }

    // hardware helper functions

    public void brake()
    {
        robotdrive_.brake();
    }

    public void drive(double speed, double turnrate)
    {
        robotdrive_.arcadeDrive(speed, turnrate);
    }

    public void driveLock(double distance)
    {
        robotdrive_.drivelock(distance);
    }

    public void spinwheel(double speed)
    {
        // if (speed == 0) {
        //     colorWheel_.selectProfileSlot(1, 0);
        //     colorWheel_.setSelectedSensorPosition(0);
        //     colorWheel_.set(ControlMode.Position, 0);
        //     // colorWheel_.set(ControlMode.PercentOutput, 0);
        // }
        // else{
            colorWheel_.set(ControlMode.PercentOutput, speed / 3);
            // colorWheel_.selectProfileSlot(0, 0);
            // colorWheel_.set(ControlMode.Velocity, speed);
            // intakeposition_ = colorWheel_.getSelectedSensorPosition();
       // }
    } 
    public void fipptuate(double speed)
        {
            if (speed == 0)
            {
                fippyRoller_.selectProfileSlot(1, 0);
                fippyRoller_.setSelectedSensorPosition(0);
                fippyRoller_.set(ControlMode.Position, 0);
            }
            else 
                fippyRoller_.set(ControlMode.PercentOutput, speed);
           
            //climbWinch_.set(ControlMode.PercentOutput, -speed);
        }

    public void movefloor(double speed)
    {
        // floor_.set(ControlMode.PercentOutput, speed);
    }


    public void letThereBeFloor(boolean isup)
    {
        floor_.selectProfileSlot(POSITION_SLOT, 0);
        floor_.set(ControlMode.Position, 
            isup ? FLOOR_DELIVER : FLOOR_BOTTOM);
    }

    public void intake(double speed)
    {
        if (speed == 0)
            {
                intakeMotor_.selectProfileSlot(1, 0);
                intakeMotor_.setSelectedSensorPosition(0);
                intakeMotor_.set(ControlMode.Position, 0);
            }
            else {
                intakeMotor_.set(ControlMode.PercentOutput, speed);
                fipptuate(speed * INTAKE_FIPP);
                letThereBeFloor(false);
            }
    }

    public void climbwithwinch(double speed)
    {
        if (speed == 0)
        {
            climbWinch_.selectProfileSlot(1, 0);
            climbWinch_.setSelectedSensorPosition(0);
            climbWinch_.set(ControlMode.Position, 0);
        }
        else 
            climbWinch_.set(ControlMode.PercentOutput, speed);
    }
   
    public void liftsabe(double speed) 
    {
        lightSaber_.setSpeed(speed);
    }

    public void liftwheel(double direction)
    {
        wheelPosition_.setSpeed(direction);
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
        return -2;
    }

    public void checkColor()
    {
        Color detectedColor = colorSensor_.getColor();
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
    }


//   public Command getAutoDriveCommand() 
//   {

//     // Create a voltage constraint to ensure we don't accelerate too fast
//     var autoVoltageConstraint =
//         new DifferentialDriveVoltageConstraint(
//             new SimpleMotorFeedforward(ksVolts,
//                     kvVoltSecondsPerMeter,
//                     kaVoltSecondsSquaredPerMeter),
//                     kDriveKinematics,
//             10);

//     // Create config for trajectory
//     TrajectoryConfig config =
//         new TrajectoryConfig(kMaxSpeedMetersPerSecond,
//                              kMaxAccelerationMetersPerSecondSquared)
//             // Add kinematics to ensure max speed is actually obeyed
//             .setKinematics(kDriveKinematics)
//             // Apply the voltage constraint
//             .addConstraint(autoVoltageConstraint);

//     // An example trajectory to follow.  All units in meters.
//     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//         // Start at the origin facing the +X direction
//         new Pose2d(0, 0, new Rotation2d(0)),
//         // Pass through these two interior waypoints, making an 's' curve path
//         List.of(
//             new Translation2d(1, 1),
//             new Translation2d(2, -1)
//         ),
//         // End 3 meters straight ahead of where we started, facing forward
//         new Pose2d(3, 0, new Rotation2d(0)),
//         // Pass config
//         config
//     );

//     RamseteCommand ramseteCommand = new RamseteCommand(
//         exampleTrajectory,
//         robotdrive_::getPose,
//         new RamseteController(kRamseteB, kRamseteZeta),
//         new SimpleMotorFeedforward(ksVolts,
//                                    kvVoltSecondsPerMeter,
//                                    kaVoltSecondsSquaredPerMeter),
//         kDriveKinematics,
//         robotdrive_::getWheelSpeeds,
//         new PIDController(kPDriveVel, 0, 0),
//         new PIDController(kPDriveVel, 0, 0),
//         // RamseteCommand passes volts to the callback
//         robotdrive_::tankDriveVolts,
//         robotdrive_
//     );

//     // Run path following command, then stop at the end.
//     return ramseteCommand;
//   }
}
