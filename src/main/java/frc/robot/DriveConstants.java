package frc.robot;

import frc.robot.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants.*;

public final class DriveConstants {
    
    public static final double kMaxRobotAccelerationMetersPerSecPerSec = 3;
    public static final double kMaxRobotAngularAccelerationRadianssPerSecPerSec = Math.PI/4;

    public static final double MAX_VOLTAGE = 12.0;

    //  The formula for calculating the theoretical maximum velocity is:
    //  <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        SdsModuleConfigurations.MK4I_L3.getDriveReduction() *
        SdsModuleConfigurations.MK4I_L3.getWheelDiameter() * Math.PI;

    //The maximum angular velocity of the robot in radians per second.
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final SwerveDriveKinematics kRobotKinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  public static double mm_CRUISE_VELOCITY_CONSTANT = 15000;

  public static double mm_ACCELERATION_CONSTANT = 6000;


}
