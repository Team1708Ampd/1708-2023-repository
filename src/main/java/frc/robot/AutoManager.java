package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.ejml.simple.AutomaticSimpleMatrixConvert;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.DriveConstants.*;

public class AutoManager  {

    // Motion Control specifies the controllers used to manipulate swerve
    MotionControl mController = null;

    // Swerve Auto Builder for generating the auto routine
    SwerveAutoBuilder autoBuilder;

    // Team Color specifying the Color being used
    TeamColor matchColor;

    // Start position specifying the starting position of the robot
    AutoRoutine routine;

    // Event map used by the path planner
    HashMap<String, Command> autoEventMap;

    // Specify Team and position
    public AutoManager(TeamColor color, AutoRoutine autoRoutine)
    {
        matchColor = color;
        routine = autoRoutine;        
    }

    // Specify Team and position and MotionControl
    public AutoManager(TeamColor color, AutoRoutine autoRoutine, MotionControl motion)
    {
        matchColor = color;
        routine = autoRoutine;         
        mController = motion;
    }

    // Allow passing in of Motion Control
    public AutoManager withMotionControl(MotionControl motion)
    {
        mController = motion;
        return this;
    }

    // Allow passing in of Team Color
    public AutoManager withTeamColor(TeamColor color)
    {
        matchColor = color;
        return this;
    }

    // Allow passing in of Start Potion
    public AutoManager withStartPosition(AutoRoutine autoRoutine)
    {
        routine = autoRoutine; ;
        return this;
    }

    // Allow passing in of Event map
    public AutoManager withEventMap(HashMap<String, Command> events)
    {
        autoEventMap = events;
        return this;
    }

    public Command generateAuto() 
    {

        // Load the correct auto
        List<PathPlannerTrajectory> autoPath = PathPlanner.loadPathGroup(routine.toString(), new PathConstraints(4, 3));

        // Generate the Auto Command and return
        autoBuilder = new SwerveAutoBuilder(mController.getSwerveSubsystem()::getCurrentPose2d, 
                                            mController.getSwerveSubsystem()::resetPose,
                                            DriveConstants.kRobotKinematics, 
                                            mController.getPIDTranslationConstants(), 
                                            mController.getPIDAngularConstants(), 
                                            mController.getSwerveSubsystem()::drive,
                                            autoEventMap,
                                            matchColor == TeamColor.BLUE,
                                            mController.getSwerveSubsystem());
        
        // Return the fully constructed auto command
        return autoBuilder.fullAuto(autoPath);
    }

    public enum TeamColor
    {
        RED,
        BLUE
    }

    public enum AutoRoutine
    {
        BLUE1PARK{
            @Override
            public String toString()
            {
                return "Blue 1 park";
            }
        },
        BLUE1CONE{
            @Override
            public String toString()
            {
                return "Blue 1 cone park";
            }
        },
        BLUE2PARK{
            @Override
            public String toString()
            {
                return "Blue 2 park";
            }
        },
        BLUE2CONE{
            @Override
            public String toString()
            {
                return "Blue 2 cone park";
            }
        },
        BLUE3PARK{
            @Override
            public String toString()
            {
                return "Blue 3 park";
            }
        },
        BLUE3CONE{
            @Override
            public String toString()
            {
                return "Blue 3 cone park";
            }
        },
        BLUE4PARK{
            @Override
            public String toString()
            {
                return "Blue 4 park";
            }
        },
        BLUE4CONE{
            @Override
            public String toString()
            {
                return "Blue 4 cone park";
            }
        },
        BLUESTRAIGHT{
            @Override
            public String toString()
            {
                return "Test";
            }
        },
        NEWPATH{
            @Override
            public String toString()
            {
                return "Auto Path";
            }
        },
    }
};



