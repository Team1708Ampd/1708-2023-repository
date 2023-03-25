package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ArmRotationSub;

public class TiltArmCommand extends CommandBase{

     double time;
     double DELAY_TIME = 1;
     boolean inverted = false;

     // The rotating arm subsystem
     ArmRotationSub armSub;

     public TiltArmCommand(double time, boolean invert, ArmRotationSub arm)
     {
          inverted = invert;

          DELAY_TIME = time;

          armSub = arm;

          addRequirements(arm);
     }

     @Override
     public void initialize()
     {
          if (inverted)
          {
          armSub.setArmOutput(-0.2);
          }
          else
          {
          armSub.setArmOutput(0.2);
          }
          

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
          armSub.setArmOutput(0);
     } 
    
}
