package frc.robot.commands; // Matches your current setup

import com.revrobotics.spark.SparkMax; // 2025 package
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.StringCoderReader;

public class ElevatorSubsystem extends SubsystemBase {
    public static class ElevatorConstants {
        public static final int spark_channel_nine = 9;
        public static final int spark_channel_ten = 10;
        public static final int coder_port = 0;
        public static final double voltage_to_distance_factor = 80/5;
        public static final double elevator_speed = 0.35;
        public static final double error = 0.5;
    }

    private final SparkMax elevator_spark_nine;
    private final SparkMax elevator_spark_ten;
    private final StringCoderReader coder;
    private final double elevator_speed;
    private final double error;
    private double current_height;

    public ElevatorSubsystem() {
        elevator_spark_nine = new SparkMax(ElevatorConstants.spark_channel_nine, MotorType.kBrushless);
        elevator_spark_ten = new SparkMax(ElevatorConstants.spark_channel_ten, MotorType.kBrushless);

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

    public void setElevatorSpeed(double speed) {
        elevator_spark_nine.set(speed);     // Motor 9: Positive up, negative down
        elevator_spark_ten.set(-speed);     // Motor 10: Negative up, positive down
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