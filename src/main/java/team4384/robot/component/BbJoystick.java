package team4384.robot.component;

import edu.wpi.first.wpilibj.Joystick;

public class BbJoystick extends Joystick {
    public static final String LOGITECH_GAMEPAD_F310 = "Controller (Gamepad F310)";
    public static final String LOGITECH_EXTREME_3D_PRO = "Logitech Extreme 3D";

    private static boolean[] oldButtonStates = new boolean[10];
    /*
     * Construct an instance of a joystick. The joystick index is the USB port on the drivers
     * station.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */

    public BbJoystick(int port) {
        super(port);
    }

    /*
    public static int findJoyStickPort(String joystickName) {
        String joyModel = joystickName.replaceAll("\\W+", "");
        System.out.println("Entered joystick Model: " + joyModel);

        DriverStation ds = DriverStation.getInstance();

        for (int i = 0; i <= 5; i++) {
            String dsJoystick = ds.getJoystickName(i).replaceAll("\\W+", "");
            System.out.println("Found Joystick Name: " + dsJoystick);
            if (joyModel.equalsIgnoreCase(dsJoystick)) {
                System.out.println("Found joystick, " + joyModel + " on port " + i);
                return i;
            }
        }

        DriverStation.reportWarning("Failed to find joystick, " + joyModel, false);
        return 5;
    }
    */

    public double getRawAxis(int axis, double deadBand) {
        if (deadBand > Math.abs(getRawAxis(axis))) {
            return 0;
        }
        return getRawAxis(axis);
    }

    public boolean isPressed(int rawButton) {
        return !oldButtonStates[rawButton] && getRawButton(rawButton);
    }

    public boolean isReleased(int rawButton) {
        return oldButtonStates[rawButton] && !getRawButton(rawButton);
    }

    public void replaceOld() {
        for (int counter = 0; counter < 10; counter++) {
            oldButtonStates[counter] = getRawButton(counter);
        }
    }
}