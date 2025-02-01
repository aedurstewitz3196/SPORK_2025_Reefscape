package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.util.Units;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.stream.Collectors;

/** Simulated IO implementation for Limelight vision system using AdvantageScope AprilTag emulation. */
public class VisionIOLimelightSim implements VisionIO {

    private final String name;
    private final SwerveDriveSimulation drive;
    private final Transform3d robotToCamera;
    private final AprilTagFieldLayout fieldLayout;
    private double horizontalFOV = 30.0;
    private double verticalFOV = 20.0;

    private final VisionIOInputs inputs = new VisionIOInputs();

    public VisionIOLimelightSim(String name, SwerveDriveSimulation drive, Transform3d robotToCamera) {
        this.name = name;
        this.drive = drive;
        this.robotToCamera = robotToCamera;
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Simulate connected status
        inputs.connected = true;

        // Get robot's current simulated pose and convert from Pose2d to Pose3d
        Pose3d robotPose = new Pose3d(drive.getSimulatedDriveTrainPose()); 

        // Find visible tags
        List<edu.wpi.first.apriltag.AprilTag> visibleTags = getTagsInRange(robotPose, 3.0); 

        if (!visibleTags.isEmpty()) {
            Logger.recordOutput("Vision/DetectedTags", visibleTags.stream()
                .map(tag -> "ID: " + tag.ID + ", Pose: " + tag.pose)
                .collect(Collectors.joining("; ")));

            // Compute weighted average pose from all visible tags
            Pose3d estimatedPose = computeWeightedAveragePose(visibleTags).transformBy(robotToCamera.inverse());

            double[] poseArray = LimelightHelpers.pose3dToArray(estimatedPose);

            // Simulate latency and timestamp for pose update
            double latency = 30.0; // Example: 30ms latency
            double timestamp = RobotController.getFPGATime() * 1.0e-6; // Convert to seconds

            // Construct the array format expected by NetworkTables
            double[] networkTablePose = new double[] {
                estimatedPose.getTranslation().getX(),
                estimatedPose.getTranslation().getY(),
                estimatedPose.getTranslation().getZ(),
                Units.radiansToDegrees(estimatedPose.getRotation().getX()),
                Units.radiansToDegrees(estimatedPose.getRotation().getY()),
                Units.radiansToDegrees(estimatedPose.getRotation().getZ()),
                latency, // Latency
                visibleTags.size(), // Tag count
                1, // Tag span (not used, but keeping format consistent)
                estimatedPose.getTranslation().getDistance(robotPose.getTranslation()), // Avg tag distance
                100, // Avg tag area (simulated value)
                visibleTags.get(0).ID // First tag ID for reference
            };

            // Publish to NetworkTables under botpose_wpiblue
            NetworkTableInstance.getDefault().getTable("limelight-sim")
                .getEntry("botpose_wpiblue").setDoubleArray(networkTablePose);

            // Simulate pose observations for AdvantageScope logging
            inputs.poseObservations = new PoseObservation[] {
                new PoseObservation(
                    timestamp - latency * 1.0e-3,  // Adjusted timestamp
                    estimatedPose,                 // Estimated pose
                    0.0,                           // Ambiguity
                    visibleTags.size(),            // Number of detected tags
                    estimatedPose.getTranslation().getDistance(robotPose.getTranslation()), // Distance
                    PoseObservationType.MEGATAG_1 // Type
                )
            };

            // Simulate tag IDs
            inputs.tagIds = visibleTags.stream().mapToInt(tag -> tag.ID).toArray();
        } else {
            inputs.latestTargetObservation = null;
            inputs.poseObservations = new PoseObservation[0];
            inputs.tagIds = new int[0];

            // Clear NetworkTables if no valid pose
            NetworkTableInstance.getDefault().getTable("limelight-sim")
                .getEntry("botpose_wpiblue").setDoubleArray(new double[0]);
        }
    }

    /**
     * Computes a weighted average pose from multiple visible AprilTags.
     * Closer tags contribute more weight to the final estimated pose.
     */
    private Pose3d computeWeightedAveragePose(List<edu.wpi.first.apriltag.AprilTag> tags) {
        if (tags.isEmpty()) return new Pose3d(); // Return default if no tags

        double totalWeight = 0.0;
        double weightedX = 0.0, weightedY = 0.0, weightedZ = 0.0;
        double weightedYaw = 0.0, weightedPitch = 0.0, weightedRoll = 0.0;

        for (var tag : tags) {
            double weight = 1.0 / tag.pose.getTranslation().getNorm(); // Closer tags contribute more
            totalWeight += weight;

            weightedX += tag.pose.getTranslation().getX() * weight;
            weightedY += tag.pose.getTranslation().getY() * weight;
            weightedZ += tag.pose.getTranslation().getZ() * weight;

            weightedYaw += tag.pose.getRotation().getZ() * weight;
            weightedPitch += tag.pose.getRotation().getY() * weight;
            weightedRoll += tag.pose.getRotation().getX() * weight;
        }

        return new Pose3d(
            weightedX / totalWeight,
            weightedY / totalWeight,
            weightedZ / totalWeight,
            new Rotation3d(weightedRoll / totalWeight, weightedPitch / totalWeight, weightedYaw / totalWeight)
        );
    }

    /**
     * Gets tags within a given range of the robot's pose.
     */
    public List<edu.wpi.first.apriltag.AprilTag> getTagsInRange(Pose3d robotPose, double maxRange) {
        return fieldLayout.getTags().stream()
                .filter(tag -> robotPose.getTranslation().getDistance(tag.pose.getTranslation()) <= maxRange)
                .sorted((tag1, tag2) -> Double.compare(
                        robotPose.getTranslation().getDistance(tag1.pose.getTranslation()),
                        robotPose.getTranslation().getDistance(tag2.pose.getTranslation())))
                .collect(Collectors.toList());
    }

    public void setFieldOfView(double horizontalFOV, double verticalFOV) {
        this.horizontalFOV = horizontalFOV;
        this.verticalFOV = verticalFOV;
    }
}
