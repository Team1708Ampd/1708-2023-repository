package frc.robot.subsystems;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.AutoConstants;

/*
 * Subsystem implementing autonomous navigation to follow pre-defined 
 * paths around the game arena. Utilizes the WPILib theta controller to 
 * provide independent heading while the swerve controller is utilized
 * to follow a generated trajectory
 */

public class NavigationSubsystem extends SubsystemBase{
    
/*
 * Instance of the drive subsystem utilized to operate the swerve drive
 */
private DriveSubsystem m_drive = null;

/*
 * Trajectory configuration needed to generate trajectories for the robot
 * to follow. 
 */
private TrajectoryConfig m_trajectoryConfig = null;

/*
 * Constructor utilized to inject dependencies needed by the subsystem. Sets up 
 * all required configurations needed to implement the swerve controller. 
 */
public NavigationSubsystem(DriveSubsystem drive
                            ){
    m_drive = drive;


    /*
     * Set up the trajectory configuration
     */
    m_trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
                                              AutoConstants.kMaxAccelerationMetersPerSecondSquared);  
}
}