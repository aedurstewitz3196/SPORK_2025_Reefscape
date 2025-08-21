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

import frc.robot.GlobalConstants.driveConstants;

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

        absoluteEncoder = new CANcoder(
            switch (module) {
                case 0 -> driveConstants.frontLeftEncoderCanId;
                case 1 -> driveConstants.frontRightEncoderCanId;
                case 2 -> driveConstants.backLeftEncoderCanId;
                case 3 -> driveConstants.backRightEncoderCanId;
                default -> 0;
            }
        );

        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getEncoder();
        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        configureCANcoder();
        // Configure conversion factors BEFORE setting the initial position so setPosition uses radians
        configureTurnMotor();
        configureDriveMotor();
        initializeTurnOffset();
    }

    private void configureCANcoder() {
        var config = new com.ctre.phoenix6.configs.CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        absoluteEncoder.getConfigurator().apply(config);
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

    private void configureTurnMotor() {
        var turnConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        turnConfig
            .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(driveConstants.turnMotorCurrentLimit)
            .inverted(true)
            .voltageCompensation(12.0);
    
        turnConfig.encoder
            .positionConversionFactor(driveConstants.turnEncoderPositionFactor) // Apply gear reduction correction
            .velocityConversionFactor(driveConstants.turnEncoderVelocityFactor); // Correct velocity scaling
    
        turnConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pidf(driveConstants.turnKp, driveConstants.turnKi, driveConstants.turnKd, 0.0)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(driveConstants.turnPIDMinInput, driveConstants.turnPIDMaxInput);
    
        turnSpark.configure(turnConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }    

    private void initializeTurnOffset() {
        double absolutePosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble(); // Rotations (0.0 to 1.0)
        double absolutePositionRad = absolutePosition * driveConstants.tau; // Convert to radians
        double zeroOffsetRad = zeroRotation.getRadians(); // Radians
        turnEncoder.setPosition(absolutePositionRad - zeroOffsetRad);
        System.out.println("Module " + absoluteEncoder.getDeviceID() + " absolutePosition is " + absolutePosition + " rotations, offsetted position is " + turnEncoder.getPosition() + " radians");
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
