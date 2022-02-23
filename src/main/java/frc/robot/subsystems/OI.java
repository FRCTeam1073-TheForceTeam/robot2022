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
        // zeroDriverController();
        operatorController = new XboxController(1);
    }

    public static void zeroDriverController() {
        LEFT_X_ZERO = getDriverLeftX();
        LEFT_Y_ZERO = getDriverLeftY();
        RIGHT_X_ZERO = getDriverRightX();
        RIGHT_Y_ZERO = getDriverRightY();
    }

    private static final double LEFT_X_MIN = -0.804;
    private static final double LEFT_X_MAX = 0.848;
    private static double LEFT_X_ZERO = 0;
    public static double getDriverLeftX() {
        return 2.0 * (driverController.getRawAxis(0) - (LEFT_X_MAX + LEFT_X_MIN) * 0.5) / (LEFT_X_MAX - LEFT_X_MIN) - LEFT_X_ZERO;
    }

    private static final double LEFT_Y_MIN = -0.919;
    private static final double LEFT_Y_MAX = 0.800;
    private static double LEFT_Y_ZERO = 0;
    public static double getDriverLeftY() {
        return 2.0 * (driverController.getRawAxis(1) - (LEFT_Y_MAX + LEFT_Y_MIN) * 0.5) / (LEFT_Y_MAX - LEFT_Y_MIN) - LEFT_Y_ZERO;
    }

    private static final double RIGHT_X_MIN=-0.774;
    private static final double RIGHT_X_MAX = 0.799;
    private static double RIGHT_X_ZERO = 0;
    public static double getDriverRightX() {
        return 2.0 * (driverController.getRawAxis(3) - (RIGHT_X_MAX + RIGHT_X_MIN) * 0.5) / (RIGHT_X_MAX - RIGHT_X_MIN) - RIGHT_X_ZERO;
    }

    private static final double RIGHT_Y_MIN = -0.598;
    private static final double RIGHT_Y_MAX = 0.700;
    private static double RIGHT_Y_ZERO = 0;
    public static double getDriverRightY() {
        return 2.0 * (driverController.getRawAxis(4) - (RIGHT_Y_MAX + RIGHT_Y_MIN) * 0.5) / (RIGHT_Y_MAX - RIGHT_Y_MIN) - RIGHT_Y_ZERO;
    }

}