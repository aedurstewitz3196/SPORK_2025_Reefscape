package frc.robot.util;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.vision.VisionIO;

/** Utility class for executing predefined robot actions. */
public class RobotActions {

    public static void logClosestAprilTag(VisionIO vision) {
        if (vision != null) {
            VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
            vision.updateInputs(inputs);

            if (inputs.poseObservations.length > 0) {
                VisionIO.PoseObservation closestTag = inputs.poseObservations[0];
                Logger.recordOutput("RobotActions/ClosestTag/ID", closestTag.type().ordinal());
                Logger.recordOutput("RobotActions/ClosestTag/Pose", new double[]{
                    closestTag.pose().getTranslation().getX(),
                    closestTag.pose().getTranslation().getY(),
                    closestTag.pose().getTranslation().getZ()
                });
                System.out.println("Closest AprilTag: ID = " + closestTag.type().ordinal() +
                    ", Pose = " + closestTag.pose());
            } else {
                System.out.println("No AprilTags detected.");
            }
        } else {
            System.out.println("Vision subsystem is null.");
        }
    }
}
