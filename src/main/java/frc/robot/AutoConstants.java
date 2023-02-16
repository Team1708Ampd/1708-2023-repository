package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.swervelib.SdsModuleConfigurations;
import static frc.robot.Constants.*;

public final class AutoConstants {

    public static double kMaxSpeedMetersPerSecond = 6380.0 / 60.0 *
    SdsModuleConfigurations.MK4I_L3.getDriveReduction() *
    SdsModuleConfigurations.MK4I_L3.getWheelDiameter() * Math.PI;

    public static double kMaxAccelerationMetersPerSecondSquared = 0.2;

    public static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );


    
}
