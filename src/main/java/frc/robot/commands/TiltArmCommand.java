package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TiltArmCommand extends CommandBase{

    double time;
    double DELAY_TIME = 1;

   public TiltArmCommand()
   {
        addRequirements(Robot.armSub);
   }

   @Override
   public void initialize()
   {
        Robot.armSub.setArmRotation(-0.2);

        time = Timer.getFPGATimestamp();
   }

   @Override
   public void execute()
   {

   }

   @Override
   public boolean isFinished()
   {
        boolean rc = false;

        if (Timer.getFPGATimestamp() > (time + DELAY_TIME))
        {
            rc = true;
        }

        return rc;
   }

   @Override
   public void end(boolean interrupted)
   {
        Robot.armSub.setArmRotation(0);
   } 
    
}
