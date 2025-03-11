// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Updated Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and a CANCoder absolute encoder for initialization.
 */
public class ModuleIOSpark implements ModuleIO {

    private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkBase driveSpark;
    private final SparkBase turnSpark;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final CANcoder absoluteEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    private long lastLogTime = 0;
    private static final long LOG_INTERVAL_MS = 1000;

    public ModuleIOSpark(int module) {
        zeroRotation = switch (module) {
            case 0 -> frontLeftZeroRotation;
            case 1 -> frontRightZeroRotation;
            case 2 -> backLeftZeroRotation;
            case 3 -> backRightZeroRotation;
            default -> new Rotation2d();
        };

        driveSpark = new SparkMax(
            switch (module) {
                case 0 -> frontLeftDriveCanId;
                case 1 -> frontRightDriveCanId;
                case 2 -> backLeftDriveCanId;
                case 3 -> backRightDriveCanId;
                default -> 0;
            },
            MotorType.kBrushless
        );

        turnSpark = new SparkMax(
            switch (module) {
                case 0 -> frontLeftTurnCanId;
                case 1 -> frontRightTurnCanId;
                case 2 -> backLeftTurnCanId;
                case 3 -> backRightTurnCanId;
                default -> 0;
            },
            MotorType.kBrushless
        );

        absoluteEncoder = new CANcoder(
            switch (module) {
                case 0 -> frontLeftEncoderCanId;
                case 1 -> frontRightEncoderCanId;
                case 2 -> backLeftEncoderCanId;
                case 3 -> backRightEncoderCanId;
                default -> 0;
            }
        );

        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getEncoder();
        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        // Configure drive motor
        configureDriveMotor();

        // Configure turn motor
        configureTurnMotor();

        // Configure CANcoder
        configureCANcoder();

        // Initialize turn offset
        initializeTurnOffset();
    }

    private void configureCANcoder() {
        var config = new com.ctre.phoenix6.configs.CANcoderConfiguration();
        
        // SET 
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        
        // IGNORE this setting if you're not actively using the CANcoder for position
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        // APPLY configuration to the CANcoder
        absoluteEncoder.getConfigurator().apply(config);

        // DISABLE updates from the CANcoder as they're no longer needed
        //absoluteEncoder.getAbsolutePosition().setUpdateFrequency(0);
        //absoluteEncoder.getVelocity().setUpdateFrequency(0);
    }

    private void configureDriveMotor() {
        var driveConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        driveConfig
            .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(driveMotorCurrentLimit)
            .inverted(false)
            .voltageCompensation(12.0);

        driveConfig.encoder
            .positionConversionFactor(driveEncoderPositionFactor)
            .velocityConversionFactor(driveEncoderVelocityFactor);

        driveConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pidf(driveKp, driveKi, driveKd, 0.0);

        driveSpark.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    private void configureTurnMotor() {
        var turnConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        turnConfig
            .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(turnMotorCurrentLimit)
            .inverted(turnEncoderInverted)
            .voltageCompensation(12.0);
    
        turnConfig.encoder
            .positionConversionFactor(turnEncoderPositionFactor) // Apply gear reduction correction
            .velocityConversionFactor(turnEncoderVelocityFactor); // Correct velocity scaling
    
        turnConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pidf(turnKp, turnKi, turnKd, 0.0)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput);
    
        turnSpark.configure(turnConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }    

    private void initializeTurnOffset() {
        double absolutePosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble(); // Rotations (0.0 to 1.0)
        double absolutePositionRad = absolutePosition * tau; // Convert to radians
        double zeroOffsetRad = zeroRotation.getRadians(); // Radians
        turnEncoder.setPosition(absolutePositionRad - zeroOffsetRad);
        //System.out.println("Module " + absoluteEncoder.getDeviceID() + " absolutePosition is " + absolutePosition + " rotations, offsetted position is " + turnEncoder.getPosition() + " radians");
    }
    private Rotation2d getTurnPosition() {
        return Rotation2d.fromRadians(turnEncoder.getPosition());
    }

    @Override
    public void setTurnPosition(Rotation2d desiredRotation) {
        double setpoint = MathUtil.inputModulus(
            desiredRotation.getRadians(), turnPIDMinInput, turnPIDMaxInput
        );
        turnController.setReference(setpoint, ControlType.kPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.turnPosition = getTurnPosition();

        long currentTime = System.currentTimeMillis();
        // Log every 1s regardless of input (temporary workaround)
        if (currentTime - lastLogTime >= LOG_INTERVAL_MS) {
            //double absolutePosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
            //System.out.println("Module " + absoluteEncoder.getDeviceID() + 
            //" absolutePosition: " + absolutePosition + " rotations, " +
            //" turn position: " + getTurnPosition().getRadians() + " radians");
            lastLogTime = currentTime;
        }
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double feedforward = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
        driveController.setReference(
            velocityRadPerSec,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            feedforward,
            com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage
        );
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition() * driveEncoderPositionFactor,
            getTurnPosition()
        );
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity() * driveEncoderVelocityFactor,
            getTurnPosition()
        );
    }

    public void setDesiredState(SwerveModuleState state) {
        setDriveVelocity(state.speedMetersPerSecond);
        setTurnPosition(state.angle);
    }
}
