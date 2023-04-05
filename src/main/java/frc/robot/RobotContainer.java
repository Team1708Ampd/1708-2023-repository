// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.pathplanner.lib.auto.PIDConstants;
import frc.robot.swervelib.ctre.CanCoderFactoryBuilder.Direction;
import pabeles.concurrency.ConcurrencyOps.Reset;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.DriveConstants.*;
import frc.robot.AutoConstants.*;
import frc.robot.AutoManager.AutoRoutine;
import frc.robot.AutoManager.TeamColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmRotationSub;
import frc.robot.subsystems.ArmTelescopingSub;
import frc.robot.subsystems.CameraSub;
import frc.robot.subsystems.WristSub;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.InvertIntake;
import frc.robot.commands.ManualArmDown;
import frc.robot.commands.ManualArmIn;
import frc.robot.commands.ManualArmOut;
import frc.robot.commands.ManualArmUp;
import frc.robot.commands.ManualWristDown;
import frc.robot.commands.ManualWristUp;
import frc.robot.commands.NavigateToAprilTagCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.PlatformBalanceCommand;
import frc.robot.commands.ResetFOD;
import frc.robot.commands.TiltArmCommand;
import frc.robot.commands.zeroGyro;
import frc.robot.commands.OuttakeAutoCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final DriveSub driveSub = new DriveSub(AutoConstants.odo_BluePositionStart6);
  public final XboxController controller = new XboxController(0);
  private final XboxController controller2 = new XboxController(1);

  private MotionControl m_MotionControl;
  private AutoManager m_AutoManager;
  private CameraSub s_camSub;
  private IntakeSub s_intake;
  private ArmRotationSub  s_ArmRotation;
  private ArmTelescopingSub s_ArmTele;
  private WristSub s_wrist;
  private SendableChooser<Integer> autoChooser;
  private SendableChooser<Integer> teamChooser;
  private boolean tilting = false;
  double speed = -1;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Init the routines manager 
    initCompetitionShuffleboard();  

    // init the robot arm so nothing breaks
    initRobotSubs();        
    
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
    new JoystickButton(controller2, XboxController.Button.kA.value).whileTrue(new IntakeCommand(s_intake));
    new JoystickButton(controller2, XboxController.Button.kB.value).whileTrue(new OuttakeCommand(s_intake));

    new JoystickButton(controller2, XboxController.Button.kLeftBumper.value).whileTrue(new ManualWristUp(s_wrist));
    new JoystickButton(controller2, XboxController.Button.kRightBumper.value).whileTrue(new ManualWristDown(s_wrist));

    new JoystickButton(controller2, XboxController.Button.kBack.value).whileTrue(new ManualArmIn(s_ArmTele));
    new JoystickButton(controller2, XboxController.Button.kStart.value).whileTrue(new ManualArmOut(s_ArmTele));

    new JoystickTrigger(controller2, XboxController.Axis.kLeftTrigger.value).whileTrue(new ManualArmUp(s_ArmRotation));
    new JoystickTrigger(controller2, XboxController.Axis.kRightTrigger.value).whileTrue(new ManualArmDown(s_ArmRotation));
    AprilTag targetTag = s_camSub.GetAprilTagFromID(6);

    new JoystickButton(controller, XboxController.Button.kA.value).onTrue(new PlatformBalanceCommand(driveSub));
    new JoystickButton(controller, XboxController.Button.kB.value).onTrue(new zeroGyro(driveSub));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Initialize the AutoRoutines that control robotic movement
    initAutoRoutines();

    if (getAutoSelecton() == AutoRoutine.BASIC)
    {
      SequentialCommandGroup autoCMD = new SequentialCommandGroup(new TiltArmCommand(1, true, s_ArmRotation), 
                                                                  new OuttakeAutoCommand(s_intake));
      return autoCMD;
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

  private void initRobotSubs()
  {
    /******** CREATE DRIVE **********/
    driveSub.setDefaultCommand(new DriveCommand(
      driveSub,
      () -> -modifyAxis(controller.getLeftY()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getLeftX()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getRightX()) * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    )); 

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
    
    // Arm Rotation
    s_ArmRotation = new ArmRotationSub(8, 9, 4)
                          .withTalonConfig(armConfig)
                          .withEncoderConfiguration(cancoderConfig);

    // Arm Extension
    s_ArmTele = new ArmTelescopingSub(10)
                      .withTalonConfig(armConfig);

    /******** CREATE WRIST **********/

    s_intake = new IntakeSub(11);

    s_wrist = new WristSub(12);

    /******** CREATE CAMERA **********/
    s_camSub = new CameraSub(driveSub, getTeamSelecton());
  }

  private void initAutoRoutines()
  {
    double autoSpeed = 2;
    m_MotionControl = new MotionControl()
      .withTranslationPIDConstants(new PIDConstants(AutoConstants.kPIDXController, 0, 0))
      .withAngularPIDConstants(new PIDConstants(AutoConstants.kPIDThetaController, 0, 0))
      .withSwerveSubsystem(driveSub); 

    AutoRoutine autoR = getAutoSelecton();

    if (autoR != AutoRoutine.BASIC)
    {
      HashMap<String, Command> eventsMap = new HashMap<>();
      eventsMap.put("balance", new PlatformBalanceCommand(driveSub));
      eventsMap.put("outtake", new OuttakeAutoCommand(s_intake));
      eventsMap.put("tiltArm", new TiltArmCommand(1, true, s_ArmRotation));
      eventsMap.put("pickArmMove", new TiltArmCommand(2.3, false, s_ArmRotation));
      eventsMap.put("zeroGyro", new ResetFOD(driveSub));
      
      m_AutoManager = new AutoManager(getTeamSelecton(), autoR)
                              .withMotionControl(m_MotionControl)
                              .withEventMap(eventsMap)
                              .withMaxSpeed(autoSpeed);
    }      
  }

  private void initCompetitionShuffleboard()
  {
    autoChooser = new SendableChooser<Integer>();
    autoChooser.setDefaultOption("Left", 1);
    autoChooser.addOption("Center", 2);
    autoChooser.addOption("Right", 3);
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