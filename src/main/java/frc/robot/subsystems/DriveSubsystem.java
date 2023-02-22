// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.swervelib.MkSwerveModuleBuilder;
import frc.robot.swervelib.SwerveModule;
import frc.robot.swervelib.SdsModuleConfigurations;
import frc.robot.swervelib.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {

  public static final double MAX_VOLTAGE = 12.0;
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L3.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L3.getWheelDiameter() * Math.PI;
  
//The maximum angular velocity of the robot in radians per second.
// Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // cause the angle reading to increase until it wraps back over to zero.
  private final Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID, "Hannibal the CANibal");
//  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

private final MkSwerveModuleBuilder m_mk4iModuleBuilder = new MkSwerveModuleBuilder();

  private String m_Canbus = "Hannibal the CANibal";

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DriveSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    zeroGyroscope();

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    m_frontLeftModule = m_mk4iModuleBuilder.withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR, m_Canbus)
                                           .withGearRatio(SdsModuleConfigurations.MK4I_L3)
                                           .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                          .withSize(2, 4)
                                                          .withPosition(0,0))
                                           .withSteerMotor(MotorType.FALCON, FRONT_LEFT_MODULE_STEER_MOTOR, m_Canbus)
                                           .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER, m_Canbus)
                                           .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
                                           .build();

   m_frontRightModule = m_mk4iModuleBuilder.withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR, m_Canbus)
                                           .withGearRatio(SdsModuleConfigurations.MK4I_L3)
                                           .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                          .withSize(2, 4)
                                                          .withPosition(0,0))
                                           .withSteerMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_STEER_MOTOR, m_Canbus)
                                           .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER, m_Canbus)
                                           .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
                                           .build();

    m_backLeftModule = m_mk4iModuleBuilder.withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR, m_Canbus)
                                           .withGearRatio(SdsModuleConfigurations.MK4I_L3)
                                           .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                          .withSize(2, 4)
                                                          .withPosition(0,0))
                                           .withSteerMotor(MotorType.FALCON, BACK_LEFT_MODULE_STEER_MOTOR, m_Canbus)
                                           .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER, m_Canbus)
                                           .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
                                           .build();

    m_backRightModule = m_mk4iModuleBuilder.withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR, m_Canbus)
                                           .withGearRatio(SdsModuleConfigurations.MK4I_L3)
                                           .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                          .withSize(2, 4)
                                                          .withPosition(0,0))
                                           .withSteerMotor(MotorType.FALCON, BACK_RIGHT_MODULE_STEER_MOTOR, m_Canbus)
                                           .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER, m_Canbus)
                                           .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
                                           .build();
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    m_pigeon.setYaw(0.0);
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    System.out.printf("Yaw: %f\n", m_pigeon.getYaw());
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }
}