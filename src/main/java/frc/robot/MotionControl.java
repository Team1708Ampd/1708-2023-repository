package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.DriveConstants.*;

public class MotionControl {

    private PIDController pid_xController;

    private PIDController pid_yController;

    private ProfiledPIDController pid_ThetaController;

    private DriveSubsystem m_drive;

    // Trajectory Configuration - All Generated Trajectories will be based on this configuration
    private TrajectoryConfig m_TrajectoryConfig;


    public MotionControl()
    {

    }


    public MotionControl withSwerveXController(PIDController xControl)
    {
        this.pid_xController = xControl;
        return this;
    }

    public MotionControl withSwerveYController(PIDController yControl)
    {
        this.pid_yController = yControl;
        return this;
    }

    public MotionControl withThetaController(ProfiledPIDController thetaControl)
    {
        this.pid_ThetaController = thetaControl;
        return this;
    }

    public MotionControl withSwerveSubsystem(DriveSubsystem drive)
    {
        this.m_drive = drive;
        return this;
    }

    public MotionControl withTrajectoryConfig(TrajectoryConfig trajConfig)
    {
        this.m_TrajectoryConfig = trajConfig;
        return this;
    }

    public Trajectory getNextTrajectory(AutoTrajectory aTrajectory)
    {

        return TrajectoryGenerator.generateTrajectory(
                aTrajectory.getStartPose(), 
                aTrajectory.getTranslationsList(), 
                aTrajectory.getEndPose(), 
                m_TrajectoryConfig
        );
    }

    public DriveSubsystem getSwerveSubsystem()
    {
        return this.m_drive;
    }

    public SwerveControllerCommand getNewSwerveCmd(AutoTrajectory aTrajectory)
    {


        return new SwerveControllerCommand(getNextTrajectory(aTrajectory), 
                m_drive::getCurrentPose2d, 
                DriveConstants.kRobotKinematics, 
                pid_xController, 
                pid_yController,
                pid_ThetaController,
                m_drive::drive, 
                m_drive);
    }

}
