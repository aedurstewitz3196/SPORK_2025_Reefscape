package frc.robot;

import frc.robot.games.reefscape2025.subsystems.drive.DriveConstants;

public class GlobalConstants {
    private GlobalConstants() {}

    // Sub-constants will be accessed as nested classes here
    public static class driveConstants extends DriveConstants {
        /*
        This class abstracts the game constants to allow access from
        the common libraries. To update or add constants files, simply
        import the additional constants classes and extend in the same way.
        */
    }
}