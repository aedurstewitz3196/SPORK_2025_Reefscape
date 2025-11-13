package frc.robot.games.reefscape2025; // Matches your current setup

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.subsystems.stringcoder.StringCoderReader;

public class ElevatorSubsystem extends SubsystemBase {
    public static class ElevatorConstants {
        public static final int coder_port = 0;
        public static final double voltage_to_distance_factor = 80 / 5;
        public static final double elevator_speed = 0.4;
        public static final double error = 0.5;
    }

    private final StringCoderReader coder;
    private final double elevator_speed;
    private final double error;
    private double current_height;

    public ElevatorSubsystem() {
        coder = new StringCoderReader(ElevatorConstants.coder_port, ElevatorConstants.voltage_to_distance_factor);
        elevator_speed = ElevatorConstants.elevator_speed;
        error = ElevatorConstants.error;
        current_height = coder.getDistance();
    }

    @Override
    public void periodic() {
        updateHeight();
    }

    public void updateHeight() {
        current_height = coder.getDistance();
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
}