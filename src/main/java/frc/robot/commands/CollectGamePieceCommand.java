package frc.robot.commands;

import java.util.function.IntToDoubleFunction;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmRotationSub;
import frc.robot.subsystems.ArmTelescopingSub;
import frc.robot.subsystems.CameraSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.WristSub;

public class CollectGamePieceCommand {

    DriveSub drive;
    CameraSub camera;
    ArmRotationSub armRot;
    ArmTelescopingSub armTele;
    WristSub wrist;
    IntakeSub intake;

    double armAngle = 1.5;
    double wristAngle = 57.0;


    public CollectGamePieceCommand(DriveSub drv, 
                                   CameraSub cam, 
                                   ArmRotationSub rot, 
                                   ArmTelescopingSub tele,
                                   WristSub wr,
                                   IntakeSub in)
    {
        drive = drv;
        cam = cam;
        armRot = rot;
        armTele = tele;
        wrist = wr;
        intake = in;
    }


    public Command getCommand()
    {
        // Command to move arm and wrist into position
        ParallelCommandGroup moveArm = new ParallelCommandGroup(new ArmSetPositionCommand(armRot, armAngle, false), 
                                                                new WristSetPositionCommand(wrist, wristAngle, false));

        // Navigate to the game piece and activate intake to collect
        // SequentialCommandGroup collectPiece = new SequentialCommandGroup(new NavToGamepieceCommand(camera, drive, false),
        //                                       new TimedIntakeCommand(intake, 1));

        // Now mix and mesh em to together for delicious gamepiece collecting action        
        return moveArm;//new SequentialCommandGroup(moveArm, collectPiece);
    }    
}