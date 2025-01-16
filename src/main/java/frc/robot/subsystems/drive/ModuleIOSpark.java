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
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Updated Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {

    private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkBase driveSpark;
    private final SparkBase turnSpark;
    private final RelativeEncoder driveEncoder;
    private final CANcoder turnEncoder;

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
        turnEncoder = new CANcoder(
            switch (module) {
                case 0 -> frontLeftEncoderCanId;
                case 1 -> frontRightEncoderCanId;
                case 2 -> backLeftEncoderCanId;
                case 3 -> backRightEncoderCanId;
                default -> 0;
            }
        );
        
        driveEncoder = driveSpark.getEncoder();
        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();
      
        // Configure drive motor
        configureDriveMotor();

        // Configure turn motor
        configureTurnMotor();

        // Configure turn motor encoder
        configureCANcoder();
    }

    private void configureCANcoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        // Apply the configuration to the CANcoder
        turnEncoder.getConfigurator().apply(config);
    }

    private void configureDriveMotor() {
        var driveConfig = new SparkMaxConfig();
    
        driveConfig
            .idleMode(IdleMode.kBrake)                              // Set the idle mode to brake
            .smartCurrentLimit(driveMotorCurrentLimit)              // Apply current limiting
            .voltageCompensation(12.0);                             // Enable voltage compensation
    
        driveConfig
            .encoder
            .positionConversionFactor(driveEncoderPositionFactor)   // Set position conversion factor
            .velocityConversionFactor(driveEncoderVelocityFactor);  // Set velocity conversion factor
    
        driveConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)         // Use the encoder as feedback
            .pidf(driveKp, 0.0, driveKd, 0.0);                      // Set PIDF values
    
        tryUntilOk(
            driveSpark,
            5,
            () -> driveSpark.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        );
    }
    
    private void configureTurnMotor() {
        var turnConfig = new SparkMaxConfig();
    
        turnConfig
            .inverted(turnInverted)                                 // Set motor inversion
            .idleMode(IdleMode.kBrake)                              // Set idle mode to brake
            .smartCurrentLimit(turnMotorCurrentLimit)               // Apply current limiting
            .voltageCompensation(12.0);                             // Enable voltage compensation
    
        turnConfig
            .absoluteEncoder
            .inverted(turnEncoderInverted)                         // Set encoder inversion
            .positionConversionFactor(turnEncoderPositionFactor)   // Set position conversion factor
            .velocityConversionFactor(turnEncoderVelocityFactor);  // Set velocity conversion factor
    
        turnConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)       // Use the absolute encoder as feedback
            .positionWrappingEnabled(true)                        // Enable position wrapping
            .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)  // Set position wrapping range
            .pidf(turnKp, 0.0, turnKd, 0.0);                      // Set PIDF values
    
        tryUntilOk(
            turnSpark,
            5,
            () -> turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        );
    }    

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();

        // Update turn inputs
        inputs.turnPosition = getCANangle().minus(zeroRotation);
    }

    private Rotation2d getCANangle() {
        return Rotation2d.fromRadians(turnEncoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double feedforward = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
        driveController.setReference(
            velocityRadPerSec,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            feedforward,
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput
        );
        turnController.setReference(setpoint, ControlType.kPosition);
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition() * driveEncoderPositionFactor,
            getCANangle()
        );
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity() * driveEncoderVelocityFactor,
            getCANangle()
        );
    }
}
