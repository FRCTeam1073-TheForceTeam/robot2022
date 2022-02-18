package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class OI 
{

    public static Joystick driverController;
    public static XboxController operatorController;

    public static void init() 
    {
        driverController = new Joystick(0);
        operatorController = new XboxController(1);
    }

}