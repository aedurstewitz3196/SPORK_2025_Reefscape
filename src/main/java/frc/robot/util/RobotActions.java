package frc.robot.util;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO;

/** Utility class for executing predefined robot actions. */
public class RobotActions {

    public static void executeDockToClosestAprilTag(Drive drive, VisionIO vision) {
        DockingController dockingController = new DockingController(vision, drive);
        dockingController.initiateDocking();
    }

}
