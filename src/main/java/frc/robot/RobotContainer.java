// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualArmDown;
import frc.robot.commands.ManualArmIn;
import frc.robot.commands.ManualArmOut;
import frc.robot.commands.ManualArmUp;
import frc.robot.commands.ManualWristDown;
import frc.robot.commands.ManualWristUp;
import frc.robot.commands.OuttakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem driveSub = new DriveSubsystem();
  private final XboxController controller = new XboxController(0);
  private final XboxController controller2 = new XboxController(1);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    driveSub.setDefaultCommand(new DriveCommand(
      driveSub,
      () -> -modifyAxis(controller.getLeftY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getLeftX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getRightX()) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
));
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
    // new Button(controller::getAButton).whenPressed(driveSub::zeroGyroscope);
    new JoystickButton(controller2, XboxController.Axis.kLeftTrigger.value).whileTrue(new IntakeCommand());
    new JoystickButton(controller2, XboxController.Axis.kRightTrigger.value).whileTrue(new OuttakeCommand());

    new JoystickButton(controller2, XboxController.Button.kX.value).whileTrue(new ManualWristUp());
    new JoystickButton(controller2, XboxController.Button.kY.value).whileTrue(new ManualWristDown());

    new JoystickButton(controller2, XboxController.Button.kLeftBumper.value).whileTrue(new ManualArmUp());
    new JoystickButton(controller2, XboxController.Button.kRightBumper.value).whileTrue(new ManualArmDown());

    new JoystickButton(controller2, XboxController.Button.kBack.value).whileTrue(new ManualArmIn());
    new JoystickButton(controller2, XboxController.Button.kStart.value).whileTrue(new ManualArmOut());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(AutoConstants.m_kinematics);


    return new InstantCommand();
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
}