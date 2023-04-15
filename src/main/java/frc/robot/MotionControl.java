package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSub;
import frc.robot.DriveConstants.*;

public class MotionControl {

    private PIDConstants pid_TranslationConst;

    private PIDConstants pid_AngleConst;

    private DriveSub m_drive;

    // Trajectory Configuration - All Generated Trajectories will be based on this configuration
    private TrajectoryConfig m_TrajectoryConfig;

    // Empty Constructor
    public MotionControl()
    {

    }


    public MotionControl withTranslationPIDConstants(PIDConstants transConsts)
    {
        this.pid_TranslationConst = transConsts;
        return this;
    }

    public MotionControl withAngularPIDConstants(PIDConstants angleConsts)
    {
        this.pid_AngleConst = angleConsts;
        return this;
    }

    public MotionControl withSwerveSubsystem(DriveSub drive)
    {
        this.m_drive = drive;
        return this;
    }

    public MotionControl withTrajectoryConfig(TrajectoryConfig trajConfig)
    {
        this.m_TrajectoryConfig = trajConfig;
        return this;
    }

    // Get the PID Constants
    public PIDConstants getPIDTranslationConstants()
    {
        return pid_TranslationConst;
    }
    
    // Get the PID Constants
    public PIDConstants getPIDAngularConstants()
    {
        return pid_AngleConst;
    }

    public DriveSub getSwerveSubsystem()
    {
        return this.m_drive;
    }
}
