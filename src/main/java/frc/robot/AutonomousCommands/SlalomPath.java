// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutonomousCommands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.DriveSubsystem.DriveSubsystem;

public class SlalomPath extends InstantCommand {
  /** Creates a new SlalomPath. */
  DriveSubsystem m_robotDrive;
  public SlalomPath(DriveSubsystem robotDrive) {
    m_robotDrive = robotDrive;
   
    // Use addRequirements() here to declare subsystem dependencies.
  }

  //Called when the commmand is initally scheduled
  @Override
  public void execute() {
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(DriveConstants.kVoltageFeedForward,
                                              DriveConstants.kDriveKinematics,
                                              10.0);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                              DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                              // Add kinematics to ensure max speed is actually obeyed
                            .setKinematics(DriveConstants.kDriveKinematics)
                            // Apply the voltage constraint
                            .addConstraint(autoVoltageConstraint);
                            
    //Slalom Path
    String trajectoryJSON = "paths/Slalom.wpilib.json";
    Trajectory slalomTrajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      slalomTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    RamseteCommand ramseteCommand = new RamseteCommand(
      slalomTrajectory,
      m_robotDrive::getPose,
      new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
            DriveConstants.kVoltageFeedForward,
            DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive
      );

    // Reset odometry to the starting pose of the trajectory.  
    m_robotDrive.resetOdometry(slalomTrajectory.getInitialPose());    
    // Runs the slalom pathfinding command following command, then stop at the end.
    ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
}
