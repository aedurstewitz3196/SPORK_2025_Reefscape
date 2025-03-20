package frc.robot.util;

public class ControllerProfiles {

    // Inner class to hold the configuration for a controller profile
    public static class ControllerProfile {
        public final int leftYAxis;
        public final int leftXAxis;
        public final int rightXAxis;
        public final int buttonX;
        public final int buttonB;
        public final int buttonA;
        public final int buttonY;
        public final int leftTriggerAxis;
        public final int rightTriggerAxis;

        public ControllerProfile(
                int leftYAxis,
                int leftXAxis,
                int rightXAxis,
                int buttonX,
                int buttonB,
                int buttonA,
                int buttonY,
                int leftTriggerAxis,
                int rightTriggerAxis) {
            this.leftYAxis = leftYAxis;
            this.leftXAxis = leftXAxis;
            this.rightXAxis = rightXAxis;
            this.buttonX = buttonX;
            this.buttonB = buttonB;
            this.buttonA = buttonA;
            this.buttonY = buttonY;
            this.leftTriggerAxis = leftTriggerAxis;
            this.rightTriggerAxis = rightTriggerAxis;
        }
    }

    // Predefined profiles
    // Determine controller buttons in Advantage Scope using the Joysticks tab and selecting 'Generic
    // Joystick'
    public static final ControllerProfile MACOS_XBOX = new ControllerProfile(
            1,
            0,
            2, // Joystick Axes
            3,
            2,
            1, 
            0, // Buttons
            2,
            4);

    public static final ControllerProfile WINDOWS_WIRED_XBOX = new ControllerProfile(
            1,
            0,
            4, // Joystick Axes
            4,
            2,
            1,
            0, // Buttons
            2,
            4);

        public static final ControllerProfile ROBORIO2 = new ControllerProfile(
            1,
            0,
            4, // Joystick Axes
            3,
            2,
            1,
            4, // Buttons
            2,
            3);

    // Method to detect the current profile based on the operating system
    public static ControllerProfile detectControllerProfile() {
        String os = System.getProperty("os.name").toLowerCase();

        if (os.contains("mac")) {
            System.out.println("Detected OS: MacOS. Using MACOS_BLUETOOTH_XBOX profile.");
            return MACOS_XBOX;
        } else if (os.contains("win")) {
            System.out.println("Detected OS: Windows. Using WINDOWS_WIRED_XBOX profile.");
            return WINDOWS_WIRED_XBOX;
        } else if (os.contains("linux")) {
            System.out.println("Detected OS: Linux. Using ROBORIO2 profile.");
            return ROBORIO2;
        } else {
            throw new UnsupportedOperationException("Unsupported OS: " + os);
        }
    }
}
