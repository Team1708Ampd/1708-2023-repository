package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class AutoRunner {
    

    private MotionControl m_MotionController;


    public AutoRunner()
    {

    }

    public void withAutoTrajectories(AutoTrajectory trajectories)
    {
       
    }

    public void withMotionControl(MotionControl mControl)
    {
        this.m_MotionController = mControl;
    }


    public void Execute()
    {
        

    }
    
}
