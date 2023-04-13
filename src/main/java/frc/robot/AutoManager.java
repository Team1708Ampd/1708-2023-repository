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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    // Max speed limit
    double maxSpeed = 4;

    // Specify Team and position
    public AutoManager(AutoRoutine autoRoutine)
    {
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
        routine = autoRoutine;
        return this;
    }

    // Allow passing in of Max Speed
    public AutoManager withMaxSpeed(double speed)
    {
        maxSpeed = speed ;
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
        Command autoCmd;

        // Load the correct auto
        List<PathPlannerTrajectory> path1 = PathPlanner.loadPathGroup("RightSideBlueAuto4-12-8", new PathConstraints(maxSpeed, 3));

        // Generate the Auto Command and return
        autoBuilder = new SwerveAutoBuilder(mController.getSwerveSubsystem()::getCurrentPose2d, 
                                            mController.getSwerveSubsystem()::resetPose,
                                            DriveConstants.kRobotKinematics, 
                                            mController.getPIDTranslationConstants(), 
                                            mController.getPIDAngularConstants(), 
                                            mController.getSwerveSubsystem()::drive,
                                            autoEventMap,
                                            true,
                                            mController.getSwerveSubsystem());

        if (routine == AutoRoutine.LEFT3PIECE)
        {
            List<PathPlannerTrajectory> path2 = PathPlanner.loadPathGroup(AutoRoutine.LEFT2PIECE.toString(), new PathConstraints(maxSpeed, 3));

            autoCmd = new SequentialCommandGroup(autoBuilder.fullAuto(path2), autoBuilder.fullAuto(path1));
        }
        else if (routine == AutoRoutine.RIGHT3PIECE)
        {
            List<PathPlannerTrajectory> path2 = PathPlanner.loadPathGroup(AutoRoutine.RIGHT2PIECE.toString(), new PathConstraints(maxSpeed, 3));

            autoCmd = new SequentialCommandGroup(autoBuilder.fullAuto(path2), autoBuilder.fullAuto(path1));
        }
        else
        {
            autoCmd = autoBuilder.fullAuto(path1);
        }
        
        // Return the fully constructed auto command
        return autoCmd;
    }


    public enum TeamColor
    {
        RED,
        BLUE
    }

    public enum AutoRoutine
    {
        RIGHT2PIECE{
            @Override
            public String toString()
            {
                return "LeftSideBlueAuto4-4";
            }
        },
        RIGHT2PIECEBALANCE{
            @Override
            public String toString()
            {
                return "CenterBlue4-4";
            }
        },
        RIGHT3PIECE{
            @Override
            public String toString()
            {
                return "RightSideBlueAuto4-11-3";
            }
        },
        LEFT2PIECE{
            @Override
            public String toString()
            {
                return "LeftSideBlueAuto4-4";
            }
        },
        LEFT2PIECEBALANCE{
            @Override
            public String toString()
            {
                return "CenterBlue4-4";
            }
        },
        LEFT3PIECE{
            @Override
            public String toString()
            {
                return "RightSideBlueAuto4-11-3";
            }
        },
        BASIC{
            @Override
            public String toString()
            {
                return "Basic";
            }
        },
    }
};



