package frc.robot.util;

import frc.robot.subsystems.drive.Drive;

/** Utility class for executing predefined robot actions. */
public class RobotActions {

    public static void executeDockToClosestAprilTag(Drive drive) {
        new frc.robot.util.DockingController(drive).driveToClosestAprilTag();
    }

}
