// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.DriveConstants.*;
import frc.robot.AutoConstants.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.InvertIntake;
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
  private AutoRoutines m_AutoRoutine;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    driveSub.setDefaultCommand(new DriveCommand(
      driveSub,
      () -> -modifyAxis(controller.getLeftY()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getLeftX()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getRightX()) * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    // Configure the button bindings
    configureButtonBindings();

    // Initialize the AutoRoutines that control robotic movement
    initAutoRoutines();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(controller2, XboxController.Button.kA.value).whileTrue(new IntakeCommand());
    new JoystickButton(controller2, XboxController.Button.kB.value).whileTrue(new OuttakeCommand());

    new JoystickButton(controller2, XboxController.Button.kLeftBumper.value).whileTrue(new ManualWristUp());
    new JoystickButton(controller2, XboxController.Button.kRightBumper.value).whileTrue(new ManualWristDown());

    // new JoystickButton(controller2, XboxController.Button.kLeftBumper.value).whileTrue(new ManualArmUp());
    // new JoystickButton(controller2, XboxController.Button.kRightBumper.value).whileTrue(new ManualArmDown());

    new JoystickButton(controller2, XboxController.Button.kBack.value).whileTrue(new ManualArmIn());
    new JoystickButton(controller2, XboxController.Button.kStart.value).whileTrue(new ManualArmOut());

    new JoystickButton(controller2, XboxController.Button.kRightStick.value).onTrue(new InvertIntake());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPIDThetaController, 0, 0, AutoConstants.kTrapezoidControllerConsts);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_MotionControl = new MotionControl()
      .withSwerveXController(new PIDController(AutoConstants.kPIDXController, 0, 0))
      .withSwerveYController(new PIDController(AutoConstants.kPIDYController, 0, 0)) 
      .withThetaController(thetaController)
      .withSwerveSubsystem(driveSub)
      .withTrajectoryConfig(
        new TrajectoryConfig(
          DriveConstants.kMaxRobotAccelerationMetersPerSecPerSec, 
          DriveConstants.kMaxRobotAngularAccelerationRadianssPerSecPerSec
        )
      );                       

    AutoTrajectory nextATrajectory = m_AutoRoutine.getNextAutoTrajectory();

    return m_MotionControl.getNewSwerveCmd(nextATrajectory);
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
    m_AutoRoutine = new AutoRoutines();

    AutoTrajectory nextTrajectory= new AutoTrajectory();

    nextTrajectory.setStartPose(AutoConstants.odo_BluePositionStart6);
    nextTrajectory.setEndPose(new Pose2d(4.5, 2.6, new Rotation2d(0)));


    m_AutoRoutine.addAutoTrajectory(nextTrajectory);

  }
}