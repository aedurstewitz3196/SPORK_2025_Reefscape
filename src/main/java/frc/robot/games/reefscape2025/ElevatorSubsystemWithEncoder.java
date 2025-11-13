package frc.robot.games.reefscape2025;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.subsystems.stringcoder.EncoderPositionReader;

/**
 * ElevatorSubsystem with encoder-based position feedback.
 * This is an alternative implementation that uses motor encoders instead of string coder.
 * Use this when your string coder is broken or unavailable.
 */
public class ElevatorSubsystemWithEncoder extends SubsystemBase {
    public static class ElevatorConstants {
        // Encoder configuration - adjust these values based on your elevator mechanics
        public static final double gear_ratio = 12.0; // Example: 10:1 gear ratio
        public static final double pulley_diameter_inches = 1.8880; // Example: 2-inch diameter pulley

        public static final double elevator_speed = 0.4;
        public static final double error = 0.5;
    }

    private final EncoderPositionReader positionReader;
    private final double elevator_speed;
    private final double error;
    private double current_height;

    public ElevatorSubsystemWithEncoder() {
        // Initialize the position reader (assuming it's used elsewhere)
        positionReader = new EncoderPositionReader(
            null, // No encoder since motors are removed
            ElevatorConstants.gear_ratio,
            ElevatorConstants.pulley_diameter_inches
        );

        elevator_speed = ElevatorConstants.elevator_speed;
        error = ElevatorConstants.error;
        current_height = 0.0; // Default height since motors are removed
    }

    @Override
    public void periodic() {
        updateHeight();
    }

    public void updateHeight() {
        current_height = positionReader.getDistance();
    }

    public double getHeight() {
        return current_height;
    }

    public double getError() {
        return error;
    }

    public double getElevatorSpeed() {
        return elevator_speed;
    }

    /**
     * Reset the elevator position to zero.
     * Call this when the elevator is at the bottom position.
     */
    public void resetPosition() {
        positionReader.resetZeroPosition();
        current_height = 0.0;
    }

    /**
     * Get the raw encoder position for debugging.
     *
     * @return Current encoder position in rotations
     */
    public double getRawEncoderPosition() {
        return positionReader.getEncoderPosition();
    }

    /**
     * Get the zero position for debugging.
     *
     * @return Zero position in encoder rotations
     */
    public double getZeroPosition() {
        return positionReader.getZeroPosition();
    }
}