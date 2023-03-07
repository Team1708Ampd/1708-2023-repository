// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmSetPositionCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.ArmTelescopingSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.swervelib.ctre.CanCoderFactoryBuilder.Direction;
import frc.robot.DriveConstants.*;
import frc.robot.AutoConstants.*;
import frc.robot.ArmConstants.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualArmDown;
import frc.robot.commands.ManualArmIn;
import frc.robot.commands.ManualArmOut;
import frc.robot.commands.ManualArmUp;
import frc.robot.commands.ManualWristDown;
import frc.robot.commands.ManualWristUp;
import frc.robot.commands.OuttakeCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem driveSub = new DriveSubsystem(AutoConstants.odo_BluePositionStart6);
  private final XboxController controller = new XboxController(0);
  private final XboxController controller2 = new XboxController(1);

  private MotionControl m_MotionControl;
  private ArmRotationSubsystem  s_ArmRotation;
  private ArmTelescopingSubsystem s_ArmTele;
  private ClawSubsystem s_Claw;
  private AutoManager m_AutoManager;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    ARMPIDDebug();

    driveSub.setDefaultCommand(new DriveCommand(
      driveSub,
      () -> -modifyAxis(controller.getLeftY()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getLeftX()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getRightX()) * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    

    // Initialize the robot arm. This takes care of rotation and telescoping subsystems
    initRobotArm();

    // Initialize the AutoRoutines that control robotic movement
    initAutoRoutines();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(controller2, XboxController.Button.kA.value).whileTrue(new IntakeCommand(s_Claw));
    // new JoystickButton(controller2, XboxController.Button.kB.value).whileTrue(new OuttakeCommand(s_Claw));

    // new JoystickButton(controller2, XboxController.Button.kX.value).whileTrue(new ManualWristUp(s_Claw));
    // new JoystickButton(controller2, XboxController.Button.kY.value).whileTrue(new ManualWristDown(s_Claw));

    new JoystickButton(controller2, XboxController.Button.kLeftBumper.value).whileTrue(new ManualArmUp(s_ArmRotation));
    new JoystickButton(controller2, XboxController.Button.kRightBumper.value).whileTrue(new ManualArmDown(s_ArmRotation));

    new JoystickButton(controller2, XboxController.Button.kBack.value).whileTrue(new ManualArmIn());
    new JoystickButton(controller2, XboxController.Button.kStart.value).whileTrue(new ManualArmOut());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
                       
    return new ArmSetPositionCommand(s_ArmRotation, Math.toRadians(SmartDashboard.getNumber("ARM Setpoint", 0)), 
                                      SmartDashboard.getNumber("ARM kP", 0.5),
                                      SmartDashboard.getNumber("ARM kI", 0), 
                                      SmartDashboard.getNumber("ARM kD", 0), 
                                      SmartDashboard.getNumber("ARM kV", 0), 
                                      SmartDashboard.getNumber("ARM kG", 0));

    //return m_AutoManager.generateAuto();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private void initAutoRoutines()
  {
    m_MotionControl = new MotionControl()
      .withTranslationPIDConstants(new PIDConstants(AutoConstants.kPIDXController, 0, 0))
      .withAngularPIDConstants(new PIDConstants(AutoConstants.kPIDThetaController, 0, 0))
      .withSwerveSubsystem(driveSub); 

  }


  private void initRobotArm()
  {
    /******** CREATE ARM **********/

    //Create the config for the motors. Each are equally matched here. Defaults taken from documentation
    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.supplyCurrLimit.enable = true;
    armConfig.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
    armConfig.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
    armConfig.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered

    // Build the CANCoder config
    CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
    cancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    cancoderConfig.magnetOffsetDegrees = Math.toDegrees(ArmConstants.ARM_ROTATION_OFFSET);
    cancoderConfig.sensorDirection = Direction.CLOCKWISE == ArmConstants.ARM_DIRECTION;
    cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    

    s_ArmRotation = new ArmRotationSubsystem(8, 9, 4)
                          .withTalonConfig(armConfig)
                          .withEncoderConfiguration(cancoderConfig);

    s_ArmTele = new ArmTelescopingSubsystem(10)
                      .withTalonConfig(armConfig);

    /******** CREATE WRIST **********/

    // Build the Wrist CANCoder config
    // CANCoderConfiguration wristCancoderConfig = new CANCoderConfiguration();
    // wristCancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    // wristCancoderConfig.magnetOffsetDegrees = Math.toDegrees(ArmConstants.WRIST_ROTATION_OFFSET);
    // wristCancoderConfig.sensorDirection = Direction.CLOCKWISE == ArmConstants.WRIST_DIRECTION;
    // wristCancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

    // s_Claw = new ClawSubsystem(12, 11, 0)
    //                 .withTalonConfig(armConfig)
    //                 .withEncoderConfiguration(cancoderConfig);
  }

  private void ARMPIDDebug()
  {
    SmartDashboard.putNumber("ARM kP", 0.0);
    SmartDashboard.putNumber("ARM kI", 0.0);
    SmartDashboard.putNumber("ARM kD", 0.0);
    SmartDashboard.putNumber("ARM kV", 0.0);
    SmartDashboard.putNumber("ARM kG", 0.0);
    SmartDashboard.putNumber("ARM Setpoint", 0.0);
  }
}