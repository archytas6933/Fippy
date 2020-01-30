/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase 
{
  public static double METERS_PER_ENCODER = 0.0007143;
  public static double DEGREES_PER_ENCODER = 0.055;
  public static int LEFT_CONTROL_MOTOR = 14;
  public static int LEFT_FOLLOW_MOTOR = 12;
  public static int RIGHT_CONTROL_MOTOR = 11;
  public static int RIGHT_FOLLOW_MOTOR = 13; 

  private static double MOTOR_POSITION_P = 0.3;
  private static double MOTOR_POSITION_I = 0;
  private static double MOTOR_POSITION_D = 0.2;
  private static int MOTOR_POSITION_SLOT = 1;

  public WPI_TalonSRX leftDrive_;
  public WPI_VictorSPX leftFollow_;
  public WPI_TalonSRX rightDrive_;
  public WPI_VictorSPX rightFollow_;

  private final SpeedControllerGroup m_leftMotors;
  private final SpeedControllerGroup m_rightMotors;
//  private final DifferentialDrive m_drive;

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();
  private double zeroAngle_;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;



  public DriveSubsystem() 
  {
    leftDrive_ = new WPI_TalonSRX(LEFT_CONTROL_MOTOR);
    leftDrive_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftDrive_.setSensorPhase(true);
    leftDrive_.config_kP(MOTOR_POSITION_SLOT, MOTOR_POSITION_P);
    leftDrive_.config_kI(MOTOR_POSITION_SLOT, MOTOR_POSITION_I);
    leftDrive_.config_kD(MOTOR_POSITION_SLOT, MOTOR_POSITION_D);
    leftDrive_.setNeutralMode(NeutralMode.Brake);
    leftFollow_ = new WPI_VictorSPX(LEFT_FOLLOW_MOTOR);
    //  leftFollow_.setNeutralMode(NeutralMode.Brake);
    //  leftFollow_.follow(leftDrive_);

    rightDrive_ = new WPI_TalonSRX(RIGHT_CONTROL_MOTOR);
    rightDrive_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightDrive_.setSensorPhase(true);
    rightDrive_.config_kP(MOTOR_POSITION_SLOT, MOTOR_POSITION_P);
    rightDrive_.config_kI(MOTOR_POSITION_SLOT, MOTOR_POSITION_I);
    rightDrive_.config_kD(MOTOR_POSITION_SLOT, MOTOR_POSITION_D);
    rightDrive_.setNeutralMode(NeutralMode.Brake);
    rightFollow_ = new WPI_VictorSPX(RIGHT_FOLLOW_MOTOR);
    //  rightFollow_.setNeutralMode(NeutralMode.Brake);
    //  rightFollow_.follow(rightDrive_);

    m_leftMotors = new SpeedControllerGroup(leftDrive_, leftFollow_);
    m_rightMotors = new SpeedControllerGroup(rightDrive_, rightFollow_);

    // The robot's drive
  //  m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    resetencoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    zeroHeading();

  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), 
      getLeftDistance(), getRightDistance());
  }

  public void resetencoders()
  {
    leftDrive_.setSelectedSensorPosition(0);
    rightDrive_.setSelectedSensorPosition(0);
  }

  public double getLeftDistance()
  {
    return leftDrive_.getSelectedSensorPosition() * METERS_PER_ENCODER;
  }

  public double getLeftRate()
  {
    return leftDrive_.getSelectedSensorVelocity() * METERS_PER_ENCODER;
  }

  public double getRightDistance()
  {
    return -rightDrive_.getSelectedSensorPosition() * METERS_PER_ENCODER;
  }

  public double getRightRate()
  {
    return rightDrive_.getSelectedSensorVelocity() * METERS_PER_ENCODER;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() 
  {
    return new DifferentialDriveWheelSpeeds(getLeftRate(), getRightRate());
  }


  public void resetOdometry(Pose2d pose) {
    resetencoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void arcadeDrive(double fwd, double rot) 
  {
    leftDrive_.set(ControlMode.PercentOutput, fwd + rot);
    rightDrive_.set(ControlMode.PercentOutput, -fwd + rot);        
//    m_drive.feed();
//    m_drive.arcadeDrive(fwd, rot);
  }

  public void brake()
  {
      drivelock(0);
  }

  public void turnLock(double degrees)
  {
    resetencoders();
    leftDrive_.selectProfileSlot(1, 0);
    leftDrive_.set(ControlMode.Position, degrees / DEGREES_PER_ENCODER);
    rightDrive_.selectProfileSlot(1, 0);
    rightDrive_.set(ControlMode.Position, degrees / DEGREES_PER_ENCODER);
  }

  public void drivelock(double distance)
  {
    resetencoders();
    leftDrive_.selectProfileSlot(1, 0);
    leftDrive_.set(ControlMode.Position, distance / METERS_PER_ENCODER);
    rightDrive_.selectProfileSlot(1, 0);
    rightDrive_.set(ControlMode.Position, -distance / METERS_PER_ENCODER);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
//    m_drive.feed();
  }

  public double getAverageEncoderDistance() 
  {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  public void setMaxOutput(double maxOutput) 
  {
//    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() 
  {
    // this is bad :(
    // m_gyro.reset();
    zeroAngle_ = m_gyro.getAngle();

  }

  public double getHeading() 
  {
    return Math.IEEEremainder(m_gyro.getAngle() - zeroAngle_, 360);
  }

  public double getTurnRate() 
  {
    return m_gyro.getRate();
  }
}
