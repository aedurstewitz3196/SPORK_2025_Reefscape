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
        public final int leftTriggerAxis;
        public final int rightTriggerAxis;
        public final int rightBumper;

        public ControllerProfile(
                int leftYAxis,
                int leftXAxis,
                int rightXAxis,
                int buttonX,
                int buttonB,
                int buttonA,
                int leftTriggerAxis,
                int rightTriggerAxis,
                int rightBumper) {
            this.leftYAxis = leftYAxis;
            this.leftXAxis = leftXAxis;
            this.rightXAxis = rightXAxis;
            this.buttonX = buttonX;
            this.buttonB = buttonB;
            this.buttonA = buttonA;
            this.leftTriggerAxis = leftTriggerAxis;
            this.rightTriggerAxis = rightTriggerAxis;
            this.rightBumper = rightBumper;
        }
    }

    // Predefined profiles
    // Determine controller buttons in Advantage Scope using the Joysticks tab and selecting 'Generic
    // Joystick'
    public static final ControllerProfile MACOS_XBOX = new ControllerProfile(
            1,
            0,
            2, // Joystick Axes
            4,
            2,
            1, // Buttons
            5,
            4,
            8 // Triggers and Bumper
            );

    public static final ControllerProfile WINDOWS_WIRED_XBOX = new ControllerProfile(
            1,
            0,
            4, // Joystick Axes
            3,
            2,
            1, // Buttons
            2,
            3,
            6 // Triggers and Bumper
            );

        public static final ControllerProfile ROBORIO2 = new ControllerProfile(
            1,
            0,
            4, // Joystick Axes
            3,
            2,
            1, // Buttons
            2,
            3,
            6 // Triggers and Bumper
            );

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
