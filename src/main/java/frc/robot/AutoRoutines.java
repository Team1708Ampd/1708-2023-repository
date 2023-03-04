package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.AutoTrajectory;


// Class contains the exact routine to be run by the robot during auto mode
public class AutoRoutines {
    
    // List of AutoTrajectories that will be followed during auto
    private List<AutoTrajectory> m_AutoTrajectories;

    // Iterator to traverse the list of AutoTrajectories
    private ListIterator<AutoTrajectory> m_aTrajectoryItr;

    // Holds the current pose of the robot
    private Pose2d m_CurrentPose;

    // Holds the last pose of the robot
    private Pose2d m_PreviousPose;


    // Empty constructor. 
    public AutoRoutines()
    {
        // Initialize the sCurrent and Previous pose of the robot
        // Start and end pose get x = 0, y = 0, (meters), rotation = 0 degrees radians
        m_CurrentPose = new Pose2d(0, 0, new Rotation2d(0));
        m_PreviousPose =   new Pose2d(0, 0, new Rotation2d(0));
        this.m_AutoTrajectories = new ArrayList<AutoTrajectory>();
    }

    // Constructor that adds current pose
    public AutoRoutines(Pose2d currentPose)
    {
        m_CurrentPose = currentPose;
        m_PreviousPose =   new Pose2d(0, 0, new Rotation2d(0));
        this.m_AutoTrajectories = new ArrayList<AutoTrajectory>();
    }

    // Constructor that adds current pose and an initial AutoTrajectory
    public AutoRoutines(Pose2d currentPose, AutoTrajectory aTrajectory)
    {
        this.m_AutoTrajectories = new ArrayList<AutoTrajectory>();
        m_PreviousPose =   new Pose2d(0, 0, new Rotation2d(0));

        if (currentPose != null)
        {
            m_CurrentPose = currentPose;
        }     

        if (aTrajectory != null)
        {
            m_AutoTrajectories.add(aTrajectory);

            // Update the iterator
            m_aTrajectoryItr = m_AutoTrajectories.listIterator();

            // Update the current pose to be the start pose of the Auto Trajectory
            this.m_CurrentPose = aTrajectory.getStartPose();
        }
    }

    // Get the current pose of the robot
    public Pose2d getCurrentPose()
    {
        return this.m_CurrentPose;
    }

    // Get the previous pose of the robot
    public Pose2d getPreviousPose()
    {
        return this.m_PreviousPose;
    }

    // Add AutoTrajectory to list of AutoTrajectories
    public void addAutoTrajectory(AutoTrajectory aTrajectory)
    {
        if (aTrajectory != null)
        {
            this.m_AutoTrajectories.add(aTrajectory);

            // If the list size is 1 then make this the current AutoTrajectory
            if (this.m_AutoTrajectories.size() == 1)
            {
                m_aTrajectoryItr = m_AutoTrajectories.listIterator();

                // Set the current pose based on the start pose of the Trajectory
                this.m_CurrentPose = aTrajectory.getStartPose();
            }
        }
    }

    // Gets the next AutoTrajectory and updates the current/previous pose
    public AutoTrajectory getNextAutoTrajectory()
    {
        AutoTrajectory rc = new AutoTrajectory();

        if (this.m_aTrajectoryItr.hasNext())
        {
            rc = this.m_aTrajectoryItr.next();
        }
        else
        {
            rc = null;
        }

        return rc;
    }
}
