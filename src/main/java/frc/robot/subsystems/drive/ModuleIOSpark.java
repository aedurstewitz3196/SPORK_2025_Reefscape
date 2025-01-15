// Copyright 2021-2024 FRC 6328
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

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.*;

/**
 * Module IO implementation for Spark Max motor controllers and CAN-based encoders (CANCoder).
 */
public class ModuleIOSpark implements ModuleIO {
    private final Rotation2d zeroRotation;
    private final int module;

    // Hardware objects
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;
    private final CANcoder moduleEncoder;

    private final SparkClosedLoopController driveClosedLoopController;
    private final SparkClosedLoopController turnClosedLoopController;
    private final RelativeEncoder driveEncoder;

    public ModuleIOSpark(int module) {
        this.module = module;

        System.out.println("Initializing ModuleIOSpark " + module);

        zeroRotation = switch (module) {
            case 0 -> frontLeftZeroRotation;
            case 1 -> frontRightZeroRotation;
            case 2 -> backLeftZeroRotation;
            case 3 -> backRightZeroRotation;
            default -> new Rotation2d();
        };

        // Initialize drive and turn motors
        driveMotor = new SparkMax(switch (module) {
            case 0 -> frontLeftDriveCanId;
            case 1 -> frontRightDriveCanId;
            case 2 -> backLeftDriveCanId;
            case 3 -> backRightDriveCanId;
            default -> 0;
        }, MotorType.kBrushless);
        driveClosedLoopController = driveMotor.getClosedLoopController();
        driveEncoder = driveMotor.getAlternateEncoder();
        configureSparkMax(driveMotor, true);

        turnMotor = new SparkMax(switch (module) {
            case 0 -> frontLeftTurnCanId;
            case 1 -> frontRightTurnCanId;
            case 2 -> backLeftTurnCanId;
            case 3 -> backRightTurnCanId;
            default -> 0;
        }, MotorType.kBrushless);
        turnClosedLoopController = turnMotor.getClosedLoopController();
        configureSparkMax(turnMotor, false);

        // Initialize CAN-based encoder
        moduleEncoder = new CANcoder(switch (module) {
            case 0 -> frontLeftEncoderCanId;
            case 1 -> frontRightEncoderCanId;
            case 2 -> backLeftEncoderCanId;
            case 3 -> backRightEncoderCanId;
            default -> 0;
        });
        configureCANcoder();
    }

    private void configureSparkMax(SparkMax motor, boolean isDriveMotor) {
        motor.clearFaults();
        if (isDriveMotor) {
            // Configure turn motor
            var driveConfig = new SparkMaxConfig();
            driveConfig
                    .inverted(turnInverted)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(turnMotorCurrentLimit)
                    .voltageCompensation(12.0);
                driveConfig
                    .absoluteEncoder
                    .inverted(turnEncoderInverted)
                    .positionConversionFactor(turnEncoderPositionFactor)
                    .velocityConversionFactor(turnEncoderVelocityFactor)
                    .averageDepth(2);
                driveConfig
                    .closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .positionWrappingEnabled(true)
                    .pidf(turnKp, 0.0, turnKd, 0.0);
                driveConfig
                    .signals
                    .absoluteEncoderPositionAlwaysOn(true)
                    .absoluteEncoderVelocityAlwaysOn(true)
                    .absoluteEncoderVelocityPeriodMs(20)
                    .appliedOutputPeriodMs(20)
                    .busVoltagePeriodMs(20)
                    .outputCurrentPeriodMs(20);
                motor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            // Configure turn motor
            var turnConfig = new SparkMaxConfig();
                turnConfig
                    .inverted(turnInverted)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(turnMotorCurrentLimit)
                    .voltageCompensation(12.0);
                turnConfig
                    .absoluteEncoder
                    .inverted(turnEncoderInverted)
                    .positionConversionFactor(turnEncoderPositionFactor)
                    .velocityConversionFactor(turnEncoderVelocityFactor)
                    .averageDepth(2);
                turnConfig
                    .closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .positionWrappingEnabled(true)
                    .pidf(turnKp, 0.0, turnKd, 0.0);
                turnConfig
                    .signals
                    .absoluteEncoderPositionAlwaysOn(true)
                    .absoluteEncoderVelocityAlwaysOn(true)
                    .absoluteEncoderVelocityPeriodMs(20)
                    .appliedOutputPeriodMs(20)
                    .busVoltagePeriodMs(20)
                    .outputCurrentPeriodMs(20);
                motor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    private void configureCANcoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        MagnetSensorConfigs sensorCongif = new MagnetSensorConfigs();

        sensorCongif.MagnetOffset = 0.0;
        sensorCongif.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        config.withMagnetSensor(sensorCongif);

        moduleEncoder.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        try {
            double drivePositionRot = driveEncoder.getPosition();
            double driveVelocityRotPerSec = driveEncoder.getVelocity();

            inputs.drivePositionRad = drivePositionRot * driveEncoderPositionFactor;
            inputs.driveVelocityRadPerSec = driveVelocityRotPerSec * driveEncoderVelocityFactor;

            double turnAbsolutePosition = moduleEncoder.getAbsolutePosition().getValueAsDouble();
            inputs.turnPosition = new Rotation2d(Math.toRadians(turnAbsolutePosition)).minus(zeroRotation);
        } catch (Exception e) {
            System.err.println("Error in updateInputs: " + e.getMessage());
            e.printStackTrace();
        }
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveMotor.set(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnMotor.set(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double adjustedVelocity = driveInversions[module] ? -velocityRadPerSec : velocityRadPerSec;
        driveClosedLoopController.setReference(adjustedVelocity, SparkBase.ControlType.kVelocity);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(rotation.plus(zeroRotation).getRadians(), -Math.PI, Math.PI);
        turnClosedLoopController.setReference(setpoint, SparkBase.ControlType.kPosition);
    }
}
