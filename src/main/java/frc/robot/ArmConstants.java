package frc.robot;

import frc.robot.swervelib.ctre.CanCoderFactoryBuilder.Direction;

public final class ArmConstants {

    // ARM Rotation Offset 
    public static final double ARM_ROTATION_OFFSET = Math.toRadians(146.00);

    // ARM Direction 
    public static final Direction ARM_DIRECTION = Direction.COUNTER_CLOCKWISE;

    // Wrist Rotation Offset 
    public static final double WRIST_ROTATION_OFFSET = 0;

    // Wrist Direction 
    public static final Direction WRIST_DIRECTION = Direction.CLOCKWISE;
}
