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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        // General drive motor inputs
        public boolean driveConnected = false; // Is the drive motor connected
        public double drivePositionRad = 0.0; // Drive encoder position in radians
        public double driveVelocityRadPerSec = 0.0; // Drive encoder velocity in rad/sec
        public double driveAppliedVolts = 0.0; // Voltage applied to the drive motor
        public double driveCurrentAmps = 0.0;
        public double driveSupplyCurrent = 0.0; // Supply current to the drive motor
        public double driveStatorCurrent = 0.0; // Stator current of the drive motor
        public boolean driveFault = false; // Drive motor fault state
    
        // General turn motor inputs
        public boolean turnConnected = false; // Is the turn motor connected
        public Rotation2d turnPosition = new Rotation2d(); // Turn encoder position
        public double turnVelocityRadPerSec = 0.0; // Turn encoder velocity in rad/sec
        public double turnAppliedVolts = 0.0; // Voltage applied to the turn motor
        public double turnCurrentAmps = 0.0;
        public double turnSupplyCurrent = 0.0; // Supply current to the turn motor
        public double turnStatorCurrent = 0.0; // Stator current of the turn motor
        public boolean turnFault = false; // Turn motor fault state
    
        // Absolute encoder inputs (for Phoenix6 CANcoder)
        public double moduleAbsolutePositionRad = 0.0; // Absolute position in radians
        public double moduleVelocityRadPerSec = 0.0; // Absolute encoder velocity in rad/sec
    
        // Odometry-related fields
        public double[] odometryTimestamps = new double[] {}; // Timestamps for odometry updates
        public double[] odometryDrivePositionsRad = new double[] {}; // Drive encoder positions over time
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {}; // Turn encoder positions over time
    }    

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified open loop value. */
    public default void setDriveOpenLoop(double output) {}

    /** Run the turn motor at the specified open loop value. */
    public default void setTurnOpenLoop(double output) {}

    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(double velocityRadPerSec) {}

    /** Run the turn motor to the specified rotation. */
    public default void setTurnPosition(Rotation2d rotation) {}

    /** Get the position of the current swerve module */
    default SwerveModulePosition getPosition() {
        return null;
    }

    /** Get the state of the current swerve module */
    default SwerveModuleState getState() {
        return null;
    }

}