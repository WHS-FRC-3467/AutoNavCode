/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written explicitly for pedagogical purposes - actual code should
 * inline a command this simple with {@link edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class SplitArcadeDrive extends CommandBase
{

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param rotation  The control input for turning
     */
    public SplitArcadeDrive(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation)
    {
        m_drive = subsystem;
        m_forward = forward;
        m_rotation = rotation;
        addRequirements(m_drive);
    }

  

	@Override
    public void execute()
    {
        // Use Slew Rate Limiter to calculate moderated values before calling arcadeDrive
        // Take the negative of the "forward" value, because the joystick Y-axis is negative in the "forward" direction.
        // if (RobotContainer.m_driverController.getRawButton(5)){
            
            m_drive.arcadeDrive((( m_forward.getAsDouble()) * 0.75), (-(m_rotation.getAsDouble())* 0.75));
        // }
        // else{ 
        //     m_drive.arcadeDrive((-1) * m_forward.getAsDouble(), (0.8) * m_rotation.getAsDouble());
        // }
    }

}
