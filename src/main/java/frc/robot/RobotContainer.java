// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.DriveConstants.*;
import frc.robot.AutoConstants.*;
import frc.robot.AutoManager.AutoRoutine;
import frc.robot.AutoManager.TeamColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.CameraSub;

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
import frc.robot.commands.PlatformBalanceCommand;
import frc.robot.commands.OuttakeAutoCommand;

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
  private AutoManager m_AutoManager;
  private CameraSub m_camSub;
  private SendableChooser<Integer> autoChooser;
  private SendableChooser<Integer> teamChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    // Init the routines manager 
    initCompetitionShuffleboard();

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
    if (getAutoSelecton() == AutoRoutine.BASIC)
    {
      return new OuttakeAutoCommand();
    }
    else
    {
      return m_AutoManager.generateAuto();
    }    
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

    AutoRoutine autoR = getAutoSelecton();

    if (autoR != AutoRoutine.BASIC)
    {
      HashMap<String, Command> eventsMap = new HashMap<>();
      eventsMap.put("balance", new PlatformBalanceCommand(driveSub));
      eventsMap.put("outtake", new OuttakeAutoCommand());

      m_AutoManager = new AutoManager(getTeamSelecton(), autoR)
                            .withMotionControl(m_MotionControl)
                            .withEventMap(eventsMap);
    }
  }

  private void initCompetitionShuffleboard()
  {
    autoChooser = new SendableChooser<Integer>();
    autoChooser.setDefaultOption("Auto 1", 1);
    autoChooser.addOption("Auto 2", 2);
    autoChooser.addOption("Auto 3", 3);
    autoChooser.addOption("Basic", 4);

    teamChooser = new SendableChooser<Integer>();
    teamChooser.setDefaultOption("BLUE", 1);
    teamChooser.addOption("RED", 2);

    Shuffleboard.getTab("SmartDashboard")
      .add(autoChooser);

    Shuffleboard.getTab("SmartDashboard")
      .add(teamChooser);
  }

  private AutoRoutine getAutoSelecton()
  {
    AutoRoutine routine = AutoRoutine.BASIC;
    
    switch(autoChooser.getSelected())
    {
      case 1:
        routine = AutoRoutine.BLUE1PARK;
      break;

      case 2:
      routine = AutoRoutine.BLUE2PARK;
      break;

      case 3:
      routine = AutoRoutine.BLUE3PARK;
      break;
      default:
      routine = AutoRoutine.BASIC;
      break;
    }

    return routine;
  }

  private TeamColor getTeamSelecton()
  {
    TeamColor team = TeamColor.BLUE;
    
    switch(teamChooser.getSelected())
    {
      case 1:
        team = TeamColor.BLUE;
      break;

      case 2:
        team = TeamColor.RED;
      break;

      default:
        team = TeamColor.BLUE;
      break;
    }

    return team;
  }
}