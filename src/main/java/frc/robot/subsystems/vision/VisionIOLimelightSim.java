package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.drive.Drive;

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
    Pose3d robotPose = new Pose3d(drive.getSimulatedDriveTrainPose()); // Convert Pose2d to Pose3d

    // Find visible tags
    List<edu.wpi.first.apriltag.AprilTag> visibleTags = getTagsInRange(robotPose, 3.0); // Example range of 3 meters

    if (!visibleTags.isEmpty()) {
        Logger.recordOutput("Vision/DetectedTags", visibleTags.stream()
            .map(tag -> "ID: " + tag.ID + ", Pose: " + tag.pose)
            .collect(Collectors.joining("; ")));

        // Use the closest visible tag
        edu.wpi.first.apriltag.AprilTag closestTag = visibleTags.get(0);

        // Simulate target observation
        inputs.latestTargetObservation = new TargetObservation(
                calculateTX(robotPose, closestTag.pose),
                calculateTY(robotPose, closestTag.pose)
        );

        // Simulate pose observations
        inputs.poseObservations = new PoseObservation[]{
            new PoseObservation(
                    System.currentTimeMillis() * 1.0e-3,  // Simulated timestamp
                    closestTag.pose,                     // Simulated tag pose
                    0.0,                                 // Ambiguity
                    1,                                   // Tag count
                    robotPose.getTranslation().getDistance(closestTag.pose.getTranslation()), // Distance
                    PoseObservationType.MEGATAG_1        // Observation type
            )
        };

        // Simulate tag IDs
        inputs.tagIds = new int[]{closestTag.ID};

        // Log to AdvantageScope
        Logger.recordOutput("Vision/SimulatedTagID", closestTag.ID);
        Logger.recordOutput("Vision/SimulatedTagPose", new double[]{
            closestTag.pose.getTranslation().getX(),
            closestTag.pose.getTranslation().getY(),
            closestTag.pose.getTranslation().getZ()
        });
    } else {
        inputs.latestTargetObservation = null;
        inputs.poseObservations = new PoseObservation[0];
        inputs.tagIds = new int[0];
    }
}

    /**
     * Gets tags within a given range of the robot's pose.
     */
    private List<edu.wpi.first.apriltag.AprilTag> getTagsInRange(Pose3d robotPose, double maxRange) {
        return fieldLayout.getTags().stream()
                .filter(tag -> robotPose.getTranslation().getDistance(tag.pose.getTranslation()) <= maxRange)
                // .filter(tag -> isWithinFieldOfView(robotPose, tag.pose)) // Temporarily disable FOV filtering
                .sorted((tag1, tag2) -> Double.compare(
                        robotPose.getTranslation().getDistance(tag1.pose.getTranslation()),
                        robotPose.getTranslation().getDistance(tag2.pose.getTranslation())))
                .collect(Collectors.toList());
    }

    /**
     * Calculates the horizontal offset (TX) to a tag from the robot's pose.
     */
    private Rotation2d calculateTX(Pose3d robotPose, Pose3d tagPose) {
        double dx = tagPose.getTranslation().getX() - robotPose.getTranslation().getX();
        double dy = tagPose.getTranslation().getY() - robotPose.getTranslation().getY();
        return Rotation2d.fromRadians(Math.atan2(dy, dx));
    }

    /**
     * Calculates the vertical offset (TY) to a tag from the robot's pose.
     */
    private Rotation2d calculateTY(Pose3d robotPose, Pose3d tagPose) {
        double dz = tagPose.getTranslation().getZ() - robotPose.getTranslation().getZ();
        double dx = tagPose.getTranslation().getX() - robotPose.getTranslation().getX();
        return Rotation2d.fromRadians(Math.atan2(dz, dx));
    }

    private boolean isWithinFieldOfView(Pose3d robotPose, Pose3d tagPose) {
        // Calculate the relative translation
        Pose3d relativePose = tagPose.relativeTo(robotPose);
    
        // Extract translation components
        double dx = relativePose.getTranslation().getX();
        double dy = relativePose.getTranslation().getY();
        double dz = relativePose.getTranslation().getZ();
    
        // Calculate horizontal (tx) and vertical (ty) angles in degrees
        double tx = Math.toDegrees(Math.atan2(dy, dx)); // Horizontal angle
        double ty = Math.toDegrees(Math.atan2(dz, dx)); // Vertical angle
    
        // Check if the angles fall within the field of view
        return Math.abs(tx) <= horizontalFOV && Math.abs(ty) <= verticalFOV;
    }
    
    public void setFieldOfView(double horizontalFOV, double verticalFOV) {
        this.horizontalFOV = horizontalFOV;
        this.verticalFOV = verticalFOV;
    }
    
}
