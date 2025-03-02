package frc.robot.commands;

public class ElevatorConstants{

    public static final double coder_height = 10;
    public static final double L1 = 16 - coder_height;
    public static final double L2 = 32 - coder_height;
    public static final double L3 = 48 - coder_height;
    public static final double L4 = 72 - coder_height;
    public static final double elevator_speed = 0.2;
    public static final double error = 0.1;
    public static final int spark_channel = 0;

    public static final int coder_port = 0;
    public static final int coder_length = 80;
    public static final int coder_voltage = 5;
    public static final double voltage_to_distance_factor = coder_length / coder_voltage;


    public static final int shooter_spark_channel = 1;
}