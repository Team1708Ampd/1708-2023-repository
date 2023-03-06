package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRotationSubsystem;

public class ArmSetPositionCommand extends CommandBase{

    // Feed forward controller
    private ArmFeedforward ffController;

    // PID Controller 
    private PIDController pidController;

    // Arm Rotation Subsystem
    private ArmRotationSubsystem armSubsystem;

    // PID constants
    private double kP, kI, kD;

    // Feed Forward Constants
    private double kV, kG;

    // Target angle for the command
    private double targetAngle = 0;

    boolean useSuppliedConstants = false;

    public ArmSetPositionCommand(ArmRotationSubsystem rotSubsystem, double newAngle)
    {
        armSubsystem = rotSubsystem;
        targetAngle = newAngle;

        // Add the subsystem as a requirement
        addRequirements(rotSubsystem);

        // Get the control constants
        if (!useSuppliedConstants){
            // Get the control constants
            getControlConstants();
        }

        // Init the controllers. PID gets a tolerance
        pidController = new PIDController(kP, kI, kD);
        pidController.disableContinuousInput();
        pidController.setTolerance(0.5 * Math.PI);


        ffController = new ArmFeedforward(0, kG, kV);

        // Get the first calculation and set the motors to start moving
        pidController.setSetpoint(targetAngle);

        // First iteration only uses feed forward to do the adjustment. All other iterations will feed back
        double output = (ffController.calculate(targetAngle, 1, 1) +
                         pidController.calculate(targetAngle));

        // Set the output
        armSubsystem.setArmOutput(output);
    }

    // Execute override
    @Override
    public void execute()
    {
        // Get the current angle of the Arm Subsystem
        double currentAngle = armSubsystem.getArmAngle();

        // Read the current arm angle and feedback
        armSubsystem.setArmOutput(pidController.calculate(currentAngle));
    }

    // End override
    @Override
    public void end(boolean interrupted)
    {
        // Set the arm output to no power
        armSubsystem.setArmOutput(0);
    }

    public ArmSetPositionCommand withControlConstants(double newkP, double newkI, double newkD, double newffV, double newffG)
    {
        useSuppliedConstants = true;
        kP = newkP;
        kI = newkI;
        kD = newkD;
        kV = newffV;
        kG = newffG;
        return this;
    }

    // isfinished override
    @Override
    public boolean isFinished()
    {
        // Finish when the PID tolerance limit is met        
        return pidController.atSetpoint();
    }

    private void getControlConstants()
    {
        kP = armSubsystem.getPIDkP();
        kI = armSubsystem.getPIDkI();
        kD = armSubsystem.getPIDkD();

        kV = armSubsystem.getFFkV();
        kG = armSubsystem.getFFkG();
    }
    
}
