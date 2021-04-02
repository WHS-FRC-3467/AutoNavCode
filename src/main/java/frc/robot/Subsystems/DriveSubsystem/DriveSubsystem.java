/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems.DriveSubsystem;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase
{

    // Four Falcon 500 motors with Talon FX controllers
    private final WPI_TalonFX m_leftTalon1 = new WPI_TalonFX(CanConstants.left_drivebase_1);
    private final WPI_TalonFX m_leftTalon2 = new WPI_TalonFX(CanConstants.left_drivebase_2);
    private final WPI_TalonFX m_rightTalon1 = new WPI_TalonFX(CanConstants.right_drivebase_1);
    private final WPI_TalonFX m_rightTalon2 = new WPI_TalonFX(CanConstants.right_drivebase_2);
 
    //Speed Controler Group for left motor
    private final SpeedControllerGroup m_leftMotors =
        new SpeedControllerGroup(m_leftTalon1, m_leftTalon2);

    private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightTalon1, m_rightTalon2);
    private final Encoder m_leftEncoder =
        new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1],
                  DriveConstants.kLeftEncoderReversed);

    // The right-side drive encoder
    private final Encoder m_rightEncoder =
        new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1],
                  DriveConstants.kRightEncoderReversed);

    private final PigeonIMU m_gyro = new PigeonIMU(6);
    
                  // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;
                
    
    // The robot's drive object
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftTalon1, m_rightTalon1);

    private double m_last_speed = 0.0;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem()
    {
        // Reset the Talons to factory defaults
        m_leftTalon1.configFactoryDefault();
        m_leftTalon2.configFactoryDefault();
        m_rightTalon1.configFactoryDefault();
        m_rightTalon2.configFactoryDefault();

        // Slave the second Talon on each side to the first
        m_leftTalon2.follow(m_leftTalon1);
        m_rightTalon2.follow(m_rightTalon1);

        // Configure Falcons to use integrated encoder
        m_leftTalon1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        m_rightTalon1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        m_leftTalon1.configOpenloopRamp(0.4);
        m_leftTalon2.configOpenloopRamp(0.4);        
        m_rightTalon2.configOpenloopRamp(0.4);        
        m_rightTalon1.configOpenloopRamp(0.4);

        //Sets units for distance as meters instead of encoder ticks
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    }

    @Override
    public void periodic() {
    // Update the odometry in the periodic block
        m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(),
                            m_rightEncoder.getDistance());
    }
    /**
     * Drives the robot using arcade controls.
     *
     * @param speed the commanded forward movement
     * @param curve the commanded rotation
     */

    public void arcadeDrive(double fwd, double rot)
    {
        fwd = limitAcceleration(fwd, m_last_speed, DriveConstants.slewRate);
        m_last_speed = fwd;
        m_drive.arcadeDrive(fwd, -rot);
        displayEncoderValues();
    }

    /**
     * Gets the left drive encoder value.
     *
     * @return the left drive encoder value
     */
    public int getLeftEncoderValue()
    {
        return m_leftTalon1.getSelectedSensorPosition();
    }

    /**
     * Gets the right drive encoder value.
     *
     * @return the right drive encoder value
     */
    public int getRightEncoderValue()
    {
        return m_rightTalon1.getSelectedSensorPosition();
    }

    public void displayEncoderValues()
    {
      System.out.println(getRightEncoderValue() + getLeftEncoderValue());
    }

    public WPI_TalonFX getLeftTalon()
    {
        return m_leftTalon1;
    }

    public WPI_TalonFX getRightTalon()
    {
        return m_rightTalon1;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput)
    {
        m_drive.setMaxOutput(maxOutput);
    }

    private double limitAcceleration(double input, double lastVal, double changeLimit) {
		double val = input;
		double change;
		    /*
         *  Slew rate limiter - limit rate of change
         */
    	change = val - lastVal;	
    	if (change > changeLimit)
    		change = changeLimit;
    	else if (change < -changeLimit)
    		change = -changeLimit;
    	
    	val = lastVal += change;
        
        return val;
    }
    
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
  
  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
    
  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

}
