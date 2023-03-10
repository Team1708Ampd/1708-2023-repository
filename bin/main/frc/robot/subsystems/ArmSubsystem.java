package frc.robot.subsystems;


//TODO Remove un-needed libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.swervelib.ctre.*;

import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {
    
    public ArmSubsystem(Integer talonDeviceNum1, Integer talonDeviceNum2, Integer talonDeviceNum3) {
        
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
        armMotor2 = new TalonFX(talonDeviceNum2);
        armOutMotor = new TalonFX(talonDeviceNum3);

        armEncoder = new CANCoder(4);

        armMotor1.setNeutralMode(NeutralMode.Brake);
        armMotor2.setNeutralMode(NeutralMode.Brake);
        armOutMotor.setNeutralMode(NeutralMode.Brake);

        //Create the config for the motors. Each are equally matched here. Defaults taken from documentation
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armConfig.supplyCurrLimit.enable = true;
        armConfig.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
        armConfig.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
        armConfig.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
        // //Setup PID Control
        armConfig.slot0.kP = pidProportional;
        armConfig.slot0.kI = pidIntegral;
        armConfig.slot0.kD = pidDerivative;
        
        // //Config the Current Limits for the motors 
        CtreUtils.checkCtreError(armMotor1.configAllSettings(armConfig), "Failed to configure Falcon 500: ArmMotor1");
        CtreUtils.checkCtreError(armMotor2.configAllSettings(armConfig), "Failed to configure Falcon 500: ArmMotor2");

        //Config the power (percent) output for the motors (default from docs)
        // armMotor1.set(TalonFXControlMode.PercentOutput, 0.5); // runs the motor at 50% power
        // armMotor2.set(TalonFXControlMode.PercentOutput, 0.5); // runs the motor at 50% power



    }

    @Override
    public void periodic() {
      System.out.println("Arm CANCoder absolute position: " + armEncoder.getAbsolutePosition());
    }

    public void setArm(double power) {
      armMotor1.set(TalonFXControlMode.PercentOutput, power);
      armMotor2.set(TalonFXControlMode.PercentOutput, power);
    }

    public void setArmOut(double power) {
      armOutMotor.set(ControlMode.PercentOutput, power);
    }

    //Method to print out debug info for the two Arm motors
    public void debugMotors() {
        System.out.println(armMotor1.getSelectedSensorPosition());      // prints the position of the selected sensor
        System.out.println(armMotor1.getSelectedSensorVelocity());      // prints the velocity recorded by the selected sensor
        System.out.println(armMotor1.getMotorOutputPercent());          // prints the percent output of the motor (0.5)
        System.out.println(armMotor1.getStatorCurrent());               // prints the output current of the motor
        
        System.out.println(armMotor2.getSelectedSensorPosition());      // prints the position of the selected sensor
        System.out.println(armMotor2.getSelectedSensorVelocity());      // prints the velocity recorded by the selected sensor
        System.out.println(armMotor2.getMotorOutputPercent());          // prints the percent output of the motor (0.5)
        System.out.println(armMotor2.getStatorCurrent());               // prints the output current of the motor
    }


    private TalonFX armMotor1;
    private TalonFX armMotor2;
    private TalonFX armOutMotor;
    private CANCoder armEncoder;
    

    // PID configuration
    private static final double pidProportional  = 0.2;
    private static final double pidIntegral      = 0.0;
    private static final double pidDerivative    = 0.1;
    
    // Motor Position Values
    double positionArmMotor1 = 0;
    double positionArmMotor2 = 0;

}