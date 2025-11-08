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

package frc.robot.common.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.GlobalConstants.driveConstants;

/**
 * Updated Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and a REV Through Bore Encoder absolute encoder for initialization.
 */
public class ModuleIOSpark implements ModuleIO {

    private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkBase driveSpark;
    private final SparkBase turnSpark;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    public ModuleIOSpark(int module) {
        zeroRotation = switch (module) {
            case 0 -> driveConstants.frontLeftZeroRotation;
            case 1 -> driveConstants.frontRightZeroRotation;
            case 2 -> driveConstants.backLeftZeroRotation;
            case 3 -> driveConstants.backRightZeroRotation;
            default -> new Rotation2d();
        };

        driveSpark = new SparkMax(
            switch (module) {
                case 0 -> driveConstants.frontLeftDriveCanId;
                case 1 -> driveConstants.frontRightDriveCanId;
                case 2 -> driveConstants.backLeftDriveCanId;
                case 3 -> driveConstants.backRightDriveCanId;
                default -> 0;
            },
            MotorType.kBrushless
        );

        turnSpark = new SparkMax(
            switch (module) {
                case 0 -> driveConstants.frontLeftTurnCanId;
                case 1 -> driveConstants.frontRightTurnCanId;
                case 2 -> driveConstants.backLeftTurnCanId;
                case 3 -> driveConstants.backRightTurnCanId;
                default -> 0;
            },
            MotorType.kBrushless
        );

        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getEncoder();
        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        // Get the absolute encoder from the turn Spark Max (REV Through Bore Encoder connected to data port)
        absoluteEncoder = turnSpark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        // Configure turn motor basic settings and conversion factors BEFORE setting position
        configureTurnMotorBasic();
        // Configure absolute encoder settings
        configureAbsoluteEncoder();
        // Initialize the turn offset BEFORE configuring closed-loop control
        initializeTurnOffset();
        // Configure closed-loop control AFTER position is properly set
        configureTurnMotorClosedLoop();
        configureDriveMotor();
    }

    private void configureAbsoluteEncoder() {
        // Configure the REV Through Bore Encoder (connected to SPARK MAX data port)
        // The encoder position is in rotations (0.0 to 1.0)
        // Set conversion factors to 1.0 since position is already in rotations
        absoluteEncoder.setPositionConversionFactor(1.0);
        absoluteEncoder.setVelocityConversionFactor(1.0);
        // Note: Through Bore Encoders typically don't need inversion, but adjust if needed
        // absoluteEncoder.setInverted(false);
    }

    private void configureDriveMotor() {
        var driveConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        driveConfig
            .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(driveConstants.driveMotorCurrentLimit)
            .inverted(true)
            .voltageCompensation(12.0);

        driveConfig.encoder
            .positionConversionFactor(driveConstants.driveEncoderPositionFactor)
            .velocityConversionFactor(driveConstants.driveEncoderVelocityFactor);

        driveConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pidf(driveConstants.driveKp, driveConstants.driveKi, driveConstants.driveKd, 0.0);

        driveSpark.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    private void configureTurnMotorBasic() {
        var turnConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        turnConfig
            .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(driveConstants.turnMotorCurrentLimit)
            .inverted(true)
            .voltageCompensation(12.0);
    
        turnConfig.encoder
            .positionConversionFactor(driveConstants.turnEncoderPositionFactor) // Apply gear reduction correction
            .velocityConversionFactor(driveConstants.turnEncoderVelocityFactor); // Correct velocity scaling
        
        // Configure absolute encoder (REV Through Bore Encoder connected to data port)
        turnConfig.absoluteEncoder
            .positionConversionFactor(1.0) // Position in rotations (0.0 to 1.0)
            .velocityConversionFactor(1.0); // Velocity conversion factor
    
        // Apply basic configuration without closed-loop control
        turnSpark.configure(turnConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
    
    private void configureTurnMotorClosedLoop() {
        var turnConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        
        // Configure closed-loop control after position has been properly initialized
        turnConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pidf(driveConstants.turnKp, driveConstants.turnKi, driveConstants.turnKd, 0.0)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(driveConstants.turnPIDMinInput, driveConstants.turnPIDMaxInput);
    
        // Apply only the closed-loop configuration
        turnSpark.configure(turnConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }    

    private void initializeTurnOffset() {
        double absolutePosition = absoluteEncoder.getPosition(); // Rotations (0.0 to 1.0) from REV Through Bore Encoder
        double absolutePositionRad = absolutePosition * driveConstants.tau; // Convert to radians
        double zeroOffsetRad = zeroRotation.getRadians(); // Radians
        turnEncoder.setPosition(absolutePositionRad - zeroOffsetRad);
        
        // Small delay to ensure setPosition takes effect before closed-loop configuration
        try {
            Thread.sleep(10); // 10ms delay
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        
        System.out.println("Module " + ((SparkMax) turnSpark).getDeviceId() + " absolutePosition is " + absolutePosition + " rotations, offsetted position is " + turnEncoder.getPosition() + " radians");
    }
    private Rotation2d getTurnPosition() {
        return Rotation2d.fromRadians(turnEncoder.getPosition());
    }

    @Override
    public void setTurnPosition(Rotation2d desiredRotation) {
        double setpoint = MathUtil.inputModulus(
            desiredRotation.getRadians(), driveConstants.turnPIDMinInput, driveConstants.turnPIDMaxInput
        );
        turnController.setReference(setpoint, ControlType.kPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.turnPosition = getTurnPosition();
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double feedforward = driveConstants.driveKs * Math.signum(velocityRadPerSec) + driveConstants.driveKv * velocityRadPerSec;
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
            driveEncoder.getPosition() * driveConstants.driveEncoderPositionFactor,
            getTurnPosition()
        );
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity() * driveConstants.driveEncoderVelocityFactor,
            getTurnPosition()
        );
    }

    public void setDesiredState(SwerveModuleState state) {
        setDriveVelocity(state.speedMetersPerSecond);
        setTurnPosition(state.angle);
    }
}
