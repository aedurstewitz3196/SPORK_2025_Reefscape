package frc.robot.util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StringCoderReader extends SubsystemBase {
    private final AnalogInput stringCoder; // Analog input for the string coder
    private final double voltageToDistanceFactor; // Conversion factor: volts to distance (e.g., inches or meters)

    /**
     * Constructor for the StringCoderTest subsystem.
     *
     * @param analogPort The analog input port number (0-3) for the string coder.
     * @param voltageToDistanceFactor Conversion factor (volts to distance).
     */
    public StringCoderReader(int analogPort, double voltageToDistanceFactor) {
        this.stringCoder = new AnalogInput(analogPort);
        this.voltageToDistanceFactor = voltageToDistanceFactor;
    }

    /**
     * Get the measured distance from the string coder.
     *
     * @return Distance in user-defined units (e.g., inches, meters).
     */
    public double getDistance() {
        double voltage = stringCoder.getVoltage(); // Get the current voltage
        return (voltage * voltageToDistanceFactor); // Convert voltage to distance
    }

    @Override
    public void periodic() {
        // Log the current distance for testing
        //System.out.println("String Coder Distance: " + getDistance() + " units");
    }
}
