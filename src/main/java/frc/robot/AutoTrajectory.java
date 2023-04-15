package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class AutoTrajectory {

    // Pose2d object gives the start position and heading of the robot
    private Pose2d m_StartPosition;

    // Pose2d object gives the end position and heading of the robot
    private Pose2d m_EndPosition;

    // List of 2D Translations the robot will go through to follow this trajectory
    private List<Translation2d> m_Translations;

    // Empty constructor allows manual setup of internal parameter
    public AutoTrajectory()
    {
        // Initialize the internal objects. 

        // Start and end pose get x = 0, y = 0, (meters), rotation = 0 degrees radians
        m_StartPosition = new Pose2d(0, 0, new Rotation2d(0));
        m_EndPosition =   new Pose2d(0, 0, new Rotation2d(0));
        this.m_Translations = new ArrayList<Translation2d>();
    }

    public AutoTrajectory(Pose2d startPose, Pose2d endPose, List<Translation2d> translations)
    {
        this.m_StartPosition = startPose;
        this.m_EndPosition = endPose;
        this.m_Translations = translations;
    }

    // Manually Set the start pose
    public void setStartPose(Pose2d pose)
    {
        // null check 
        if (pose != null)
        {
            this.m_StartPosition = pose;
        }        
    }

    // Manually Set the end pose
    public void setEndPose(Pose2d pose)
    {
        // null check
        if (pose != null)
        {
            this.m_EndPosition = pose;
        }        
    }
    
    // Get the start pose
    public Pose2d getStartPose()
    {
        return this.m_StartPosition;
    }

    // Get the end pose
    public Pose2d getEndPose()
    {
        return this.m_EndPosition;
    }

    // Manually add Translation to the List of Translations
    public void addTranslation(Translation2d translation)
    {
        // Null check
        if (translation != null)
        {
            m_Translations.add(translation);
        }
    }

    // Get this list of translations
    public List<Translation2d> getTranslationsList()
    {
        return this.m_Translations;
    }
}
