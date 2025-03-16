package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

public class DriveConstants {
    // Physical robot dimensions and capabilities
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(23.50);
    public static final double wheelBase = Units.inchesToMeters(23.50);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final double tau = Math.PI*2;

    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    // Zeroed rotation values for each module to account for physical offset
    // Open Advantagescope and read degree value for straight forward. Next convert that degree value to radians.
    // Depending on whether you want to zero from the left or right, add or subtract the radian value from the motor's ZeroRotation.
    // Reference value in intializer in ModuleIOSpark to determine accuracy(comment set there to indicate line)
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(tau * 0.457);   // Encoder ID 1
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(tau * 0.69); // Encoder ID 3
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(tau * 0.12);    // Encoder ID 2
    public static final Rotation2d backRightZeroRotation = new Rotation2d(tau * 0.746);   // Encoder ID 4

    // CAN IDs
    public static final int pigeonCanId = 9;

    public static final int frontLeftDriveCanId = 2;
    public static final int backLeftDriveCanId = 4;
    public static final int frontRightDriveCanId = 6;
    public static final int backRightDriveCanId = 8;

    public static final int frontLeftTurnCanId = 1;
    public static final int backLeftTurnCanId = 3;
    public static final int frontRightTurnCanId = 5;
    public static final int backRightTurnCanId = 7;

    public static final int frontLeftEncoderCanId = 1;
    public static final int frontRightEncoderCanId = 3;
    public static final int backLeftEncoderCanId = 2;
    public static final int backRightEncoderCanId = 4;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 40; // Amperes
    public static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
    public static final double driveMotorReduction = 6.75;
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
        (2 * Math.PI * wheelRadiusMeters) / driveMotorReduction; // Sensor rotations -> wheel radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI * wheelRadiusMeters) / (60.0 * driveMotorReduction); // Sensor RPM -> rad/sec

    // Drive PID configuration
    public static final double driveKp = 0.01;   // 0.06
    public static final double driveKi = 0.00;
    public static final double driveKd = 0.00;   // 0.007
    public static final double driveKs = 0.001;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.01;
    public static final double driveSimD = 0.0001;
    public static final double driveSimKs = 0.001;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final int turnMotorCurrentLimit = 40; // Amperes
    public static final double turnMotorReduction = (150/7); // ~21.4286
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = false;
    public static final double turnEncoderPositionFactor = (2 * Math.PI) / turnMotorReduction; // Sensor rotations -> radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // Sensor RPM -> rad/sec

    // PathPlanner configuration
    public static final double robotMassKg = 74.088; // Robot mass in kilograms
    public static final double wheelCOF = 1.2; // Coefficient of friction for the wheel
    public static final double robotMOI = 6.883;
    
    // Turn PID configuration
    // Drive PID and turn PID must have a certain ratio Turn:Drive should be 10:1 approximately
    // Try to set turn PID p to about 0.10 and the i and d to 0(or else it will go haywire)
    public static final double turnKp = 0.30;  // Proportional gain
    public static final double turnKi = 0.001; // Small integral to address minor steady-state errors
    public static final double turnKd = 0.00;  // Derivative to dampen oscillations    
    public static final double turnSimP = 2.00;
    public static final double turnSimD = 0.001;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // Voltage compensation
    public static final double voltageCompensation = 12.0;

    // Neutral mode configuration
    public static final boolean neutralModeBrake = true;

// Simulation configuration
    public static final RobotConfig ppConfig = new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                    wheelRadiusMeters,
                    maxSpeedMetersPerSec,
                    wheelCOF,
                    driveGearbox.withReduction(driveMotorReduction),
                    driveMotorCurrentLimit,
                    1),
            moduleTranslations);

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
        .withCustomModuleTranslations(moduleTranslations)
        .withRobotMass(Kilogram.of(robotMassKg))
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(new SwerveModuleSimulationConfig(
                driveGearbox,
                turnGearbox,
                driveMotorReduction,
                turnMotorReduction,
                Volts.of(0.1),
                Volts.of(0.1),
                Meters.of(wheelRadiusMeters),
                KilogramSquareMeters.of(0.02),
                wheelCOF));

}
