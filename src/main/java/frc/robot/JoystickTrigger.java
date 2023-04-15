package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

// Create a Joystick trigger mechanism that behaves like a button. 
// Converts the XBoxController axis command to a boolean supplier

public class JoystickTrigger extends Trigger{
    
    /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */

    public JoystickTrigger(GenericHID joystick, int TriggerID)
    {
        super(()->(joystick.getRawAxis(TriggerID) > 0));
        requireNonNullParam(joystick, "joystick", "joystick button");
    }

}
