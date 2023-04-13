// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmTelescopingSub;

public class ExtendArmCommand extends CommandBase {
  // Create Telescoping Arm subsystem reference 
  private ArmTelescopingSub armSub;

  // Boolean Suppliers
  private BooleanSupplier changeSpeedCmd;

  // Double Suppliers
  private BooleanSupplier armInCmd;
  private BooleanSupplier armOutCmd;

  private final double ARM_SPEED_MAX = 1;
  private final double ARM_SPEED_MIN = 0.3;
  

  // Speed for arm movement
  double speed = ARM_SPEED_MAX;

  /** Creates a new ManualArmDown. */
  public ExtendArmCommand(ArmTelescopingSub arm, BooleanSupplier inCmd, BooleanSupplier outCmd) {
    
    // Get the Arm
    armSub = arm;

    // Get the suppliers
    armInCmd = inCmd;
    armOutCmd = outCmd;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (armInCmd.getAsBoolean() == true)
    {
      armSub.setArmOutput(-speed);
    }
    else if (armOutCmd.getAsBoolean() == true)
    {
      armSub.setArmOutput(speed);
    }    
    else
    {
      armSub.setArmOutput(0);
    }
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
