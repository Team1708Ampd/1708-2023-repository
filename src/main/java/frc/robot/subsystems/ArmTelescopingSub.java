package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.swervelib.ctre.*;

import static frc.robot.Constants.*;

public class ArmTelescopingSub extends SubsystemBase {
    
  private TalonFX armMotor1;

  // Output power on the arm being requested
  private double armOutRequested = 0;

  // private AnalogPotentiometer potentiometer;

  // Constructor just taking motor and encoder IDS
  public ArmTelescopingSub(int talonDeviceNum1)
  {
    //Create the Two TalonFx Motors to drive the Arm
    armMotor1 = new TalonFX(talonDeviceNum1);
    // potentiometer = new AnalogPotentiometer(0);

    setMotorNeutralMode(NeutralMode.Brake);  

    /// Initialize to 0 power output. Use direct set to make sure Arm start at 0!
    armMotor1.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  // Constructor taking device IDs and motor configuration
  public ArmTelescopingSub(int talonDeviceNum1,  TalonFXConfiguration motorConfig) {
      
    //Components / Notes:
    // Two Falcon Fx Talon 500 that work together to drive the Arm angle. 
    // use like this: TalonFX motor = new TalonFX(driveConfiguration, "Hannibal the CANibal");
    // Will have encoder at top of arm bracket to tell position
    // Want to use PID controller
    //mdh note: check out the createFalcon500 factory
    // 315 to 1 ratio for arm(?)

    //Create the Two TalonFx Motors to drive the Arm
    // TalonFX armMotor1 = new TalonFX(talonDeviceNum1, "ArmMotor1");
    armMotor1 = new TalonFX(talonDeviceNum1);

    setMotorNeutralMode(NeutralMode.Brake);    
    
    /// Initialize to 0 power output. Use direct set to make sure Arm start at 0!
    armMotor1.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {    

    // Update the motor outputs to the current desired output value
    armMotor1.set(TalonFXControlMode.PercentOutput, armOutRequested);

  }

  // Allow user to pass in Talon configuration
  public ArmTelescopingSub withTalonConfig(TalonFXConfiguration motorConfig)
  {
    // Update the config
    useMotorConfig(motorConfig);
    return this;
  }  

  // Sets the Arm motor outputs
  public void setArmOutput(double power) {

    if (power > 1)
    {
      armOutRequested = 1;
    }
    else
    {
      armOutRequested = power;
    }
    // Set the requested output power of the motor
    ;
  }

  // public void setHeight(double height) {
  //   if((potentiometer.get() - height) > 1) {
  //     armMotor1.set(ControlMode.PercentOutput, 0.5);
  //   } else if((potentiometer.get() - height) < -1) {
  //     armMotor1.set(ControlMode.PercentOutput, -0.5);
  //   }
  // }

  // public boolean atHeight(double height) {
  //   if(Math.abs(potentiometer.get() - height) < 1) {
  //     return true;
  //   }
  //   return false;
  // }

  private void useMotorConfig(TalonFXConfiguration config)
  {
      CtreUtils.checkCtreError(armMotor1.configAllSettings(config), "Failed to configure Falcon 500: ArmMotor1");
  }


  // Sets the behavior for neutral position of the motors
  private void setMotorNeutralMode(NeutralMode mode)
  {
    armMotor1.setNeutralMode(mode);
  }
}
