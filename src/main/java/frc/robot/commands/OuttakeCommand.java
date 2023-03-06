// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ClawSubsystem;

public class OuttakeCommand extends CommandBase {
  // Reference to the clab subsystem
  ClawSubsystem clawSub;

  /** Creates a new IntakeCommand. */
  public OuttakeCommand(ClawSubsystem claw) {
    clawSub = claw;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clawSub.setIntakeOutput(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawSub.setIntakeOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
