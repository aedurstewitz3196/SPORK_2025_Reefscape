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
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        // Apply configuration to the CANcoder
        absoluteEncoder.getConfigurator().apply(config);

        // Adjust the update frequency for signals to avoid CANbus saturation
        absoluteEncoder.getAbsolutePosition().setUpdateFrequency(5); // 5 Hz (200ms interval)
        absoluteEncoder.getVelocity().setUpdateFrequency(5);         // 5 Hz (if velocity data is used)
    }

    private void configureDriveMotor() {
        var driveConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        driveConfig
            .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(driveMotorCurrentLimit)
            .inverted(true)
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
        // SET absolutePosition to Physical Encoders poisition
        double absolutePosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        
        // SET turnEncoder (Relative Encoder) to the absolute encoders position
        turnEncoder.setPosition(absolutePosition - zeroRotation.getRotations());
    }

    private Rotation2d getTurnPosition() {
        return Rotation2d.fromRadians(turnEncoder.getPosition());
    }

    @Override
    public void setTurnPosition(Rotation2d desiredRotation) {
        double setpoint = MathUtil.inputModulus(
            desiredRotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput
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
