// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSub extends SubsystemBase {
  /** Creates a new ClawSub. */
  CANSparkMax wristMotor;
  TalonFX intakeMotor;
  public ClawSub(Integer wristId, Integer intakeId) {
    wristMotor = new CANSparkMax(wristId, MotorType.kBrushless);
    intakeMotor = new TalonFX(intakeId);
    wristMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setIntake(double power) {
    intakeMotor.set(ControlMode.PercentOutput, power);
  }

  public void setWrist(double power) {
    wristMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
