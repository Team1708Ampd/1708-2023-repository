package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawSetPositionCommand extends CommandBase{

    // Feed forward controller
    private ArmFeedforward ffController;

    // PID Controller 
    private PIDController pidController;

    // Arm Rotation Subsystem
    private ClawSubsystem clawSubsystem;

    // PID constants
    private double kP, kI, kD;

    // Feed Forward Constants
    private double kV, kG;

    // Target angle for the command
    private double targetAngle = 0;

    // Use supplied control constants
    boolean useSuppliedConstants = false;

    public ClawSetPositionCommand(ClawSubsystem claw, double newAngle)
    {
        clawSubsystem = claw;
        targetAngle = newAngle;

        // Add the subsystem as a requirement
        addRequirements(claw);

        

        // Init the controllers. PID gets a tolerance
        
        if (!useSuppliedConstants){
            // Get the control constants
            getControlConstants();
        }

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
        clawSubsystem.setWristOutput(output);
    }

    public ClawSetPositionCommand withControlConstants(double newkP, double newkI, double newkD, double newffV, double newffG)
    {
        useSuppliedConstants = true;
        kP = newkP;
        kI = newkI;
        kD = newkD;
        kV = newffV;
        kG = newffG;
        return this;
    }

    // Execute override
    @Override
    public void execute()
    {
        // Get the current angle of the Arm Subsystem
        double currentAngle = clawSubsystem.getWristAngle();

        // Read the current arm angle and feedback
        clawSubsystem.setWristOutput(pidController.calculate(currentAngle));
    }

    // End override
    @Override
    public void end(boolean interrupted)
    {
        System.out.println("Arm at position");
        // Set the arm output to no power
        clawSubsystem.setWristOutput(0);
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
        kP = clawSubsystem.getPIDkP();
        kI = clawSubsystem.getPIDkI();
        kD = clawSubsystem.getPIDkD();

        kV = clawSubsystem.getFFkV();
        kG = clawSubsystem.getFFkG();
    }
    
}
