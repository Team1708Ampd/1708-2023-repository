package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
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

public class ArmRotationSub extends SubsystemBase {
    
  private TalonFX armMotor1;
  private CANCoder armEncoder;

  // PID configuration
  private double PIDkP = 0.0;
  private double PIDkI = 0.0;
  private double PIDkD = 0.0;

  // Feed forward configuration
  private double FFkG = 0;
  private double FFkV = 0;

  // Output power on the arm being requested
  private double armOutRequested = 0;

  // Current position of the arm in degrees
  private Rotation2d armPositionCurrent;

  // Define rotation limits for the rotation of the arm
  private final double ROTATION_LIMIT_START = 0.0;
  private final double ROTATION_LIMIT_END = 270.0;


  // Constructor just taking motor and encoder IDS
  public ArmRotationSub(int talonDeviceNum1, int encoderNum)
  {
    //Create the Two TalonFx Motors to drive the Arm
    // TalonFX armMotor1 = new TalonFX(talonDeviceNum1, "ArmMotor1");
    armMotor1 = new TalonFX(talonDeviceNum1);

    armEncoder = new CANCoder(encoderNum);

    setMotorNeutralMode(NeutralMode.Brake);  

    /// Initialize to 0 power output. Use direct set to make sure Arm start at 0!
    armMotor1.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  // Constructor taking device IDs and motor configuration
  public ArmRotationSub(int talonDeviceNum1, int encoderNum, TalonFXConfiguration motorConfig) {
      
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

    armEncoder = new CANCoder(4);

    setMotorNeutralMode(NeutralMode.Brake);    
    
    /// Initialize to 0 power output. Use direct set to make sure Arm start at 0!
    armMotor1.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {    

    //SmartDashboard.putNumber("Arm Current Angle", Rotation2d.fromRadians(getArmAngle()).getDegrees());

    // Update the motor outputs to the current desired output value
    armMotor1.set(TalonFXControlMode.PercentOutput, armOutRequested);

    // Update the current position of the Arm
    //armPositionCurrent = Rotation2d.fromRadians(getArmAngle());
  }

  // Allow user to pass in Talon configuration
  public ArmRotationSub withTalonConfig(TalonFXConfiguration motorConfig)
  {
    // Update the config
    useMotorConfig(motorConfig);
    return this;
  }  

  // Allow user to pass in the Encoder configuration
  public ArmRotationSub withEncoderConfiguration(CANCoderConfiguration cancoderConfig)
  {
    useEncoderConfig(cancoderConfig);
    return this;
  }

  // Allow user to specify PID constants for use in PID controller
  public ArmRotationSub withPIDConstants(double kP, double kI, double kD)
  {
    PIDkP = kP;
    PIDkI = kI;
    PIDkD = kD;
    return this;
  }

  // Allow user to specify Feed Forward constants for use in FF controller
  public ArmRotationSub withFeedForwardConstants(double kV, double kG)
  {
    FFkG = kG;
    FFkV = kV;
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
  }

  // Get the current position of the encoder attached to the arm 
  public double getArmAngle() {
    int ATTEMPTS = 3;
    double angle = Math.toRadians(armEncoder.getAbsolutePosition());

    ErrorCode code = armEncoder.getLastError();

    for (int i = 0; i < ATTEMPTS; i++) {
        if (code == ErrorCode.OK) break;
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) { }
        angle = Math.toRadians(armEncoder.getAbsolutePosition());
        code = armEncoder.getLastError();
    }    

    CtreUtils.checkCtreError(code, "Failed to retrieve CANcoder "+armEncoder.getDeviceID()+" absolute position after "+ATTEMPTS+" tries");

    // angle %= 2.0 * Math.PI;
    // if (angle < 0.0) {
    //     angle += 2.0 * Math.PI;
    // }

    return Math.toDegrees(angle);
  }

  // Get the current Arm Angle Reported internally to the Arm
  public Rotation2d getArmAngleInternal()
  {
    return armPositionCurrent;
  }

  // Get the PID Constants
  public double getPIDkP()
  {
    return PIDkP;
  }

  // Get the PID Constants
  public double getPIDkI()
  {
    return PIDkI;
  }

  // Get the PID Constants
  public double getPIDkD()
  {
    return PIDkD;
  }

  // Get the FF Constants
  public double getFFkG()
  {
    return FFkG;
  }

  // Get the FF Constants
  public double getFFkV()
  {
    return FFkV;
  }

  private void useMotorConfig(TalonFXConfiguration config)
  {
      CtreUtils.checkCtreError(armMotor1.configAllSettings(config), "Failed to configure Falcon 500: ArmMotor1");
  }

  private void useEncoderConfig(CANCoderConfiguration cancoderConfig)
  {
      // Init the Cancoder config
      CtreUtils.checkCtreError(armEncoder.configAllSettings(cancoderConfig, 250), "Failed to configure CANCoder");
      CtreUtils.checkCtreError(armEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 250), "Failed to configure CANCoder update rate");
  }

  // Sets the behavior for neutral position of the motors
  private void setMotorNeutralMode(NeutralMode mode)
  {
    armMotor1.setNeutralMode(mode);
  }
}