package frc.robot.commands;

import org.opencv.ml.StatModel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSub;

public class IntakePieceCommand extends CommandBase{
    
    // Subsystems
    IntakeSub intake;

    // Current Limits
    private final double INTAKE_LOW_CURRENT_LIMIT = 0.0;
    private final double INTAKE_HIGH_CURRENT_LIMIT = 150.0;

    // Error
    private final double INTAKE_CURRENT_ERROR = 0.0;

    // Timeout 
    private double timeout = 0.0;

    // Start time 
    private double startTime = 0.0;

    // Intake Tick
    private int tick = 0;

    public IntakePieceCommand(IntakeSub in, double time)
    {
        // Get inputs
        intake = in;
        timeout = time;

        addRequirements(intake);
    }

    @Override
    public void initialize()
    {
        // Set intake to full power
        intake.setIntake(-1);

        // Get the time
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished()
    {
        boolean rc = false;

        // Check for timeout enable
        if (timeout != 0.0)
        {
            if (Timer.getFPGATimestamp() > (startTime + timeout))
            {
                rc = true;
            }
        }

        // Check for current threshold limit
        if (Math.abs(intake.getCurrent()) > (INTAKE_HIGH_CURRENT_LIMIT))
        {
            tick += 1;

            if (tick > 5)
            {
                rc = true;
            }            
        }

        return rc;
    }

    @Override
    public void end(boolean interrupted)
    {
        intake.setIntake(0);
    }
}
