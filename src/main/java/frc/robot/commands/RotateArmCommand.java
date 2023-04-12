// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.ArmRotationSub;

public class RotateArmCommand extends CommandBase {
  // Create Rotation Arm subsystem reference 
  private ArmRotationSub armSub;

  // Double Suppliers
  private DoubleSupplier armUpCmd;
  private DoubleSupplier armDownCmd;

  private final double ARM_SPEED_MAX = 0.7;
  private final double ARM_SPEED_MIN = 0.3;

  // Speed for arm movement
  double speed = ARM_SPEED_MAX;

  /** Creates a new ManualArmDown. */
  public RotateArmCommand(ArmRotationSub arm, DoubleSupplier upCmd, DoubleSupplier downCmd) {
    
    // Get the Arm
    armSub = arm;

    // Get the suppliers
    armUpCmd = upCmd;
    armDownCmd = downCmd;  

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public Subsystem getSub()
  {
    return armSub;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.setArmOutput( (armUpCmd.getAsDouble() / 2.5) - (armDownCmd.getAsDouble()/ 2.5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.setArmOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
