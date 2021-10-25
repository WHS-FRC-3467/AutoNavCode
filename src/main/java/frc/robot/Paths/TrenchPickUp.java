// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Paths;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;


import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.DriveSubsystem.DriveSubsystem;

public class TrenchPickUp{
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  public Command TrenchPickUp() {  
    // Create a voltage constraint to ensure we don't accelerate too fast
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

  
    Trajectory trenchPickUp = new Trajectory();
    String trajectoryJson = "Paths/TrenchPickup.wpilib.json";
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);
      trenchPickUp = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch (IOException ex){
      DriverStation.reportError("Unable to open trajecoty" + trajectoryJson, ex.getStackTrace());
    }
    RamseteCommand ramseteCommand = new RamseteCommand(
        trenchPickUp,
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
    m_robotDrive.resetOdometry(trenchPickUp.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
}
