package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;

/*
 * Class implementing a container for the Robot's current speed, localisation, and direction state. 
 * By default the class is initialized as a singleton with the ChassisSpeed Vx and Vy set to 0 and the 
 * Pose2d X, Y, and Theta parameters set to the origin coordinates and a 0 degree angle. 
 */

public class RobotState {

    /* Singleton instance of Robot State class */
    private static RobotState m_RobotStateinstance = null;
    
    /* Number of ChassisSpeeds historical states to store */
    int m_ChassisSpeedHistSize = 20;

    /* Number of Pose2d historical states to store */
    int m_PoseHistSize = 20;

    /* Store the history of ChassisSpeeds  */
    private ChassisSpeeds[] m_ChassisSpeedsHistory;

    /* Store the history of pose positions */
    private Pose2d[] m_Pose2dHistory;

    /* Current ChassisSpeed */
    private ChassisSpeeds m_currentChassisSpeed;

    /* Current Pose2D */
    private Pose2d m_CurrentPose;

    
    /*
     * Constructor initializes the Robot State Object with the current Speed and Pose of the 
     * robot. Sets the current ChassisSpeed and Pose2d parameters to the values specified. 
     */
    private RobotState(ChassisSpeeds currentSpeed, Pose2d CurrentPose)
    {
        /* Init the current ChassisSpeed and Pose */
        m_currentChassisSpeed = currentSpeed;
        m_CurrentPose = CurrentPose;
    }

    /* Update the RobotState current ChassisSpeed and Pose. Performs boundary checking 
     * on input values. Saves the current ChassisSpeed and Pose to their respective 
     * historical tracking lists.
     */
    public void UpdateState(ChassisSpeeds newChassisSpeed, Pose2d newPose)
    {
        /* TODO JTS - Boundary Check */

        /* TODO JTS - Save the current chassis speed */

        /* TODO JTS - Save the current pose */

       
        /* Set the current Chassis Speed and Pose */
        m_currentChassisSpeed = newChassisSpeed;
        m_CurrentPose = newPose;
    }

    /* Get the current ChassisSpeed of the robot state */
    public ChassisSpeeds GetCurrentChassisSpeed()
    {
        return m_currentChassisSpeed;
    }

    /* Get the current Pose of the robot state */
    public Pose2d GetCurrentPose()
    {
         return m_CurrentPose;
    }

    /* Debug Print for the current robot state */
    public void DebugPrintState()
    {
        /* Current Chassis Speed */
        System.out.println("ChassisSpeed \n " + 
                           "--------------- \n " +
                           "Vx (m/s) - " + m_currentChassisSpeed.vxMetersPerSecond + "\n" +
                           "Vy (m/s) - " + m_currentChassisSpeed.vyMetersPerSecond + "\n");

        /* Current Pose */
        System.out.println("Pose2D \n " + 
                           "----------------------- \n " +
                           "X (m) - " + m_CurrentPose.getX() + "\n" +
                           "Y (m) - " + m_CurrentPose.getY() + "\n" +
                           "theta (degrees) - " + m_CurrentPose.getRotation() + "\n");
    }

    /* Get the Singleton Instance */
    public static RobotState GetInstance()
    {
        /* Create a new instance if it does not exist */
        if (m_RobotStateinstance == null)
        {
            m_RobotStateinstance = new RobotState(new ChassisSpeeds(), new Pose2d());
        }
         return m_RobotStateinstance;
    }
}
