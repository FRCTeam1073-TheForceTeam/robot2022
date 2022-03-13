package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI 
{
    public static final boolean debug = false;

    public static Joystick driverController;
    public static XboxController operatorController;
    public static DPadState dPadState;

    public static void init() 
    {
        driverController = new Joystick(0);
        zeroDriverController();
        operatorController = new XboxController(1);
        dPadState = DPadState.OFF;
    }

    public static void update() {
        updateOperatorDPadState();
        if (debug){
            SmartDashboard.putString("DPAD", dPadState.toString());
        }
    }

    public static void zeroDriverController() {
        //Sets all the offsets to zero, then uses whatever value it returns as the new offset.
        LEFT_X_ZERO = 0;
        LEFT_Y_ZERO = 0;
        RIGHT_X_ZERO = 0;
        RIGHT_Y_ZERO = 0;
        LEFT_X_ZERO = getDriverLeftX();
        LEFT_Y_ZERO = getDriverLeftY();
        RIGHT_X_ZERO = getDriverRightX();
        RIGHT_Y_ZERO = getDriverRightY();
    }

    private static final double LEFT_X_MIN = -0.804;
    private static final double LEFT_X_MAX = 0.850;
    private static double LEFT_X_ZERO = 0;
    public static double getDriverLeftX() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(0) - (LEFT_X_MAX + LEFT_X_MIN) * 0.5) / (LEFT_X_MAX - LEFT_X_MIN) - LEFT_X_ZERO, -1, 1);
    }

    private static final double LEFT_Y_MIN = -0.919;
    private static final double LEFT_Y_MAX = 0.800;
    private static double LEFT_Y_ZERO = 0;
    public static double getDriverLeftY() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(1) - (LEFT_Y_MAX + LEFT_Y_MIN) * 0.5) / (LEFT_Y_MAX - LEFT_Y_MIN) - LEFT_Y_ZERO, -1, 1);
    }

    private static final double RIGHT_X_MIN=-0.774;
    private static final double RIGHT_X_MAX = 0.799;
    private static double RIGHT_X_ZERO = 0;
    public static double getDriverRightX() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(3) - (RIGHT_X_MAX + RIGHT_X_MIN) * 0.5) / (RIGHT_X_MAX - RIGHT_X_MIN) - RIGHT_X_ZERO, -1, 1);
    }

    private static final double RIGHT_Y_MIN = -0.598;
    private static final double RIGHT_Y_MAX = 0.700;
    private static double RIGHT_Y_ZERO = 0;
    public static double getDriverRightY() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(4) - (RIGHT_Y_MAX + RIGHT_Y_MIN) * 0.5) / (RIGHT_Y_MAX - RIGHT_Y_MIN) - RIGHT_Y_ZERO, -1, 1);
    }
    
    public static Trigger getOperatorDPadDown() {
        return getOperatorDPadTrigger(DPadState.DOWN);
    }

    public static Trigger getOperatorDPadUp() {
        return getOperatorDPadTrigger(DPadState.UP);
    }

    public static Trigger getOperatorDPadLeft() {
        return getOperatorDPadTrigger(DPadState.LEFT);
    }

    public static Trigger getOperatorDPadRight() {
        return getOperatorDPadTrigger(DPadState.RIGHT);
    }

    private static void updateOperatorDPadState() {
        int value = operatorController.getPOV();
        if (value == -1) {
            dPadState = DPadState.OFF;
        } else if (value % 90 == 0) {
            dPadState = DPadState.fromAngle(value);
        } else if (dPadState.isOn() && Math.abs(((value - dPadState.angle) + 180) % 360 - 180) < 45) {
            //dPadState = DPadState.fromAngle(value);
        }
    }
    
    public enum DPadState {
        OFF(-1),
        DOWN(180),
        UP(0),
        LEFT(270),
        RIGHT(90);

        public int angle;

        DPadState(int angle_) {
            angle = angle_;
        }

        public static DPadState fromAngle(int angle) {
            switch (angle) {
                case 180:
                    return DPadState.DOWN;
                case 0:
                    return DPadState.UP;
                case 270:
                    return DPadState.LEFT;
                case 90:
                    return DPadState.RIGHT;
                default:
                    return DPadState.OFF;                
            }
        }
        
        public boolean isOn() {
            return (angle != -1);
        }

        public boolean equals(DPadState other) {
            return (other.angle == angle);
        }

        public String toString() {
            return name() + "(" + angle + "degrees)";
        }
    }
    
    private static Trigger getOperatorDPadTrigger(DPadState state) {
        return new Trigger(
            ()->{
                return (dPadState == state);
            }
        );
    } 
}