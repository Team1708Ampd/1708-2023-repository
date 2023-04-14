// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.pathplanner.lib.auto.PIDConstants;
import frc.robot.swervelib.ctre.CanCoderFactoryBuilder.Direction;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmKnownPositionCommand;
import frc.robot.commands.ArmSetPositionCommand;
import frc.robot.commands.CollectGamePieceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExtendArmCommand;
import frc.robot.commands.IntakePieceCommand;
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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.ArmRotationSub;
import frc.robot.subsystems.ArmTelescopingSub;
import frc.robot.subsystems.CameraSub;
import frc.robot.subsystems.WristSub;
import frc.robot.commands.RotateArmCommand;
import frc.robot.commands.RotateWristCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.NavToTagCommand;
import frc.robot.commands.PlatformBalanceCommand;
import frc.robot.commands.ResetFOD;
import frc.robot.commands.TiltArmCommand;
import frc.robot.commands.TimedIntakeCommand;
import frc.robot.commands.WristSetPositionCommand;
import frc.robot.commands.zeroGyro;
import frc.robot.commands.zeroWrist;
import frc.robot.commands.OuttakeAutoCommand;

import frc.robot.commands.ArmKnownPositionCommand.ARMPOSITION;

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

  public POVButton up = new POVButton(controller2, 0);
  public POVButton left = new POVButton(controller2, 90);
  public POVButton right = new POVButton(controller2, 180);
  public POVButton down = new POVButton(controller2, 270);


  private MotionControl m_MotionControl;
  private AutoManager m_AutoManager;
  private CameraSub s_camSub;
  private IntakeSub s_intake;
  public ArmRotationSub  s_ArmRotation;
  private ArmTelescopingSub s_ArmTele;
  public WristSub s_wrist;
  private SendableChooser<String> autoChooser;
  private ArmKnownPositionCommand armPositions;
  private boolean tilting = false;
  private double speed = -1;


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

    AprilTag targetTag = s_camSub.GetAprilTagFromID(6);

    new JoystickButton(controller, XboxController.Button.kX.value).onTrue(new ArmSetPositionCommand(s_ArmRotation, 0, true));
    new JoystickButton(controller, XboxController.Button.kY.value).onTrue(new WristSetPositionCommand(s_wrist, 0, true));
    new JoystickButton(controller, XboxController.Button.kA.value).onTrue(new NavToTagCommand(s_camSub, driveSub, targetTag, true));
    new JoystickButton(controller, XboxController.Button.kB.value).onTrue(new zeroGyro(driveSub));
    new JoystickButton(controller, XboxController.Button.kStart.value).onTrue(new zeroWrist(s_wrist));
//     up.onTrue(new TelescopeHighCone(s_ArmTele));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Initialize the AutoRoutines that control robotic movement
    initAutoRoutines();
    
    return m_AutoManager.generateAuto();  
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
    s_ArmRotation = new ArmRotationSub(8, 4)
                          .withTalonConfig(armConfig)
                          .withEncoderConfiguration(cancoderConfig);
    s_ArmRotation.setDefaultCommand(new RotateArmCommand(s_ArmRotation, 
                                    () -> controller2.getLeftTriggerAxis(), 
                                    () -> controller2.getRightTriggerAxis()));

    // Arm Extension
    s_ArmTele = new ArmTelescopingSub(10)
                      .withTalonConfig(armConfig);

    s_ArmTele.setDefaultCommand(new ExtendArmCommand(s_ArmTele, 
                                () -> controller2.getBackButton(),
                                () -> controller2.getStartButton()));

    /******** CREATE WRIST **********/

    s_intake = new IntakeSub(11);

    s_intake.setDefaultCommand(new RunIntakeCommand(s_intake, 
                                () -> controller2.getAButton(), 
                                () -> controller2.getBButton()));

    s_wrist = new WristSub(12);

    s_wrist.setDefaultCommand(new RotateWristCommand(s_wrist,
                              () -> controller2.getLeftBumper(), 
                              () -> controller2.getRightBumper()));

    /******** CREATE CAMERA **********/
    s_camSub = new CameraSub(driveSub);

    armPositions = new ArmKnownPositionCommand(s_ArmTele, s_ArmRotation, s_wrist);
  }

  private void initAutoRoutines()
  {
    double autoSpeed = 4;
    m_MotionControl = new MotionControl()
      .withTranslationPIDConstants(new PIDConstants(AutoConstants.kPIDXController, 0, 0))
      .withAngularPIDConstants(new PIDConstants(AutoConstants.kPIDThetaController, 0, 0))
      .withSwerveSubsystem(driveSub); 

    AutoRoutine autoR = getAutoSelecton();

    CollectGamePieceCommand piece = new CollectGamePieceCommand(driveSub, s_camSub, s_ArmRotation, s_ArmTele, s_wrist, s_intake);

    HashMap<String, Command> eventsMap = new HashMap<>();
    eventsMap.put("balance", new PlatformBalanceCommand(driveSub));
    eventsMap.put("outtake", new OuttakeAutoCommand(s_intake));
    eventsMap.put("zeroGyro", new ResetFOD(driveSub));
    eventsMap.put("collectPiece", piece.getCommand());
    eventsMap.put("ArmScoreLow", armPositions.getCommand(ARMPOSITION.SCORELOW));
    eventsMap.put("ArmIntakePosition", armPositions.getCommand(ARMPOSITION.INTAKEGROUND));
    eventsMap.put("ArmBalance", armPositions.getCommand(ARMPOSITION.BALANCE));
    
    m_AutoManager = new AutoManager(autoR)
                      .withMotionControl(m_MotionControl)
                      .withEventMap(eventsMap)
                      .withMaxSpeed(autoSpeed);
  }

  private void initCompetitionShuffleboard()
  {
    autoChooser = new SendableChooser<String>();    

    for (AutoRoutine routine : AutoRoutine.values())
    {
      autoChooser.addOption(routine.name(), routine.name());
    }

    autoChooser.setDefaultOption("BASIC", "BASIC");

    Shuffleboard.getTab("SmartDashboard")
      .add(autoChooser);  
  }

  private AutoRoutine getAutoSelecton()
  {
    AutoRoutine routine = AutoRoutine.BASIC;
    
    for (AutoRoutine r : AutoRoutine.values())
    {
      if (r.name() == autoChooser.getSelected())
      {
        routine = r;
      }
    }

    return routine;
  }
}
