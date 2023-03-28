package frc.robot.commands;

import java.util.stream.Stream;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSub;
import frc.robot.subsystems.DriveSub;

public class NavigateToAprilTagCommand extends CommandBase{

    // Camera 
    CameraSub camera;

    // Drive 
    DriveSub drive;

    // April tag we are navigating to
    AprilTag navTag;

    // PID controller for strafing
    PIDController strafeController = new PIDController(0.008, 0, 0);

    // PID Controller for range
    PIDController rangeController = new PIDController(0.008, 0, 0);

    double MIN_MOTOR_EFFORT = 0.1;

    public NavigateToAprilTagCommand(CameraSub cam, DriveSub drv, AprilTag tag)
    {
        camera = cam;
        drive = drv;
        navTag = tag;

        // Set the setpoint and tolerance for each PID Controller
        strafeController.setSetpoint(0); // TODO figure out setpoint
        strafeController.setTolerance(0.5); // TODO figure out tolerance

        rangeController.setSetpoint(0); // TODO figure out setpoint
        rangeController.setTolerance(0.5); // TODO figure out tolerance

        // Set the vision pipeline to use for the tag. Should correspond to the tag ID
        camera.SetLimelight1Pipeline(tag.ID);
    }

    @Override
    public void execute()
    {
        // First we get the current error with distance from the April tag
        NetworkTable lime = camera.GetLimelight1();

        // check to see if we have the target in vision
        if (lime.getEntry("tv").getDouble(0) == 1)
        {
            double xError = lime.getEntry("tx").getDouble(0);
            double yError = lime.getEntry("ty").getDouble(0);

            // Now calculate the next state of the drive output from both PID Controllers
            double strafeV = strafeController.calculate(xError);
            double rangeV = strafeController.calculate(yError);
            
            if (strafeV < MIN_MOTOR_EFFORT)
            {
                strafeV = MIN_MOTOR_EFFORT;
            }

            if (rangeV < MIN_MOTOR_EFFORT)
            {
                rangeV = MIN_MOTOR_EFFORT;
            }            

            ChassisSpeeds speed = new ChassisSpeeds(strafeV, rangeV, 0);

            drive.drive(speed);
        }
    }

    @Override
    public boolean isFinished()
    {
        boolean rc = false;

        if (rangeController.atSetpoint() && strafeController.atSetpoint())
        {
            rc = true;
        }

        return rc;
    }

    @Override
    public void end(boolean interrupted)
    {
        drive.stopBrake();
    }
    
}
