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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {
  /** Creates a new ClawSub. */
  TalonFX intakeMotor;
  public IntakeSub(Integer intakeId) {
    intakeMotor = new TalonFX(intakeId);
  }

  public void setIntake(double power) {
    intakeMotor.set(ControlMode.PercentOutput, power);
  }

  public void invertIntake() {
    if(intakeMotor.getInverted()) {
      intakeMotor.setInverted(false);
    } else {
      intakeMotor.setInverted(true);
    }
  }

  public double getCurrent()
  {
    return intakeMotor.getStatorCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("INTAKE CURRENT", getCurrent());
  }
}
