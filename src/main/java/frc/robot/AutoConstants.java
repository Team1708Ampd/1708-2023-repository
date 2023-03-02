package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.*;
import frc.robot.DriveConstants.*;

public final class AutoConstants {

    // Constants used to implement robotic odometry for the swerve drive
    // Init positions are based on field setup described below. This field 
    // setup is mirrored across the origin and thus there are a total of 6 
    // possible starting positions: 3 Red Team, 3 Blue Team. Starting position
    // headings are all defined as having the robot within the boundaries of 
    // one of the 6 starting position quadrants with the front of the robot 
    // facing the origin 
    /*
     *           ________________________________________________________
     *          |                          GOAL                          |  O = Origin
     *          |--------------------------------------------------------|
     *          |     START 1     |       START 2      |     START 3     |
     *          |                 |                    |                 |
     *          |--------------------------------------------------------|
     *          |                                                        |
     *          |             _____________________________              |
     *          |            |                             |             |
     *          |            |           PLATFORM          |             |
     *          |            |_____________________________|             |
     *          |                                                        |
     *          |                                                        |
     *          |                          |                             |
     *        --|--------------------------O-----------------------------|--
     *          |                          |                             |
     */
    
    public static final Pose2d odo_RedPositionStart1 = new Pose2d(1, 1, new Rotation2d(0.0)); // Start position A Red Team
    public static final Pose2d odo_RedPositionStart2 = new Pose2d(2, 1, new Rotation2d(0.0)); // Start position B Red Team
    public static final Pose2d odo_RedPositionStart3 = new Pose2d(3, 1, new Rotation2d(0.0)); // Start position C Red Team
    
    public static final Pose2d odo_BluePositionStart4 = new Pose2d(1, 1, new Rotation2d(0.0)); // Start position A Blue Team
    public static final Pose2d odo_BluePositionStart5 = new Pose2d(2, 1, new Rotation2d(0.0)); // Start position B Blue Team
    public static final Pose2d odo_BluePositionStart6 = new Pose2d(1.027, 4.627, new Rotation2d(0.0)); // Start position C Blue Team

    // Define a starting position for the Origin just for tessting purposes
    public static final Pose2d odo_PositionStartOrigin = new Pose2d(0, 0, new Rotation2d(0.0)); // Start position Origin



    // Performance related Constants for implementing PID Controllers

    // Robot Speeds in Auto
    public static final double kAutoMaxSpeedMetersPerSecond = DriveConstants.MAX_VELOCITY_METERS_PER_SECOND/4;
    public static final double kAutoMaxAngularSpeedRadiansPerSecond = DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND/4;
    
    public static final double kAutoMaxRobotAccelerationMetersPerSecPerSec = DriveConstants.kMaxRobotAccelerationMetersPerSecPerSec;
    public static final double kAutoMaxRobotAngularAccelerationRadiansPerSecPerSec = DriveConstants.kMaxRobotAngularAccelerationRadianssPerSecPerSec;

    public static final double kPIDXController = 1.5;
    public static final double kPIDYController = 1.5;
    public static final double kPIDThetaController = 3;

    // Trapezoidal Speed Controller Constraints
    public static final TrapezoidProfile.Constraints kTrapezoidControllerConsts = 
            new TrapezoidProfile.Constraints(
                            kAutoMaxAngularSpeedRadiansPerSecond, 
                            kAutoMaxRobotAngularAccelerationRadiansPerSecPerSec
            );

}
