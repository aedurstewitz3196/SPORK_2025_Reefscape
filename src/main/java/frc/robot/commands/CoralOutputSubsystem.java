package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralOutputSubsystem extends SubsystemBase {
    // Assuming ElevatorConstants is defined elsewhere; add shooter channel
    public static class ElevatorConstants {
        public static final int shooter_spark_channel = 11; // Define this (e.g., CAN ID 11)
    }

    private final SparkMax shooterMotor;
    private final DigitalInput funnelLazer;
    private final DigitalInput shooterLazer;
    private final DigitalInput exitLazer;
    private boolean isPulling = false;
    private boolean isShooting = false;
    private static final double PULL_SPEED = 0.5;

    public CoralOutputSubsystem() {
        shooterMotor = new SparkMax(ElevatorConstants.shooter_spark_channel, MotorType.kBrushless);
        funnelLazer = new DigitalInput(0);
        shooterLazer = new DigitalInput(1);
        exitLazer = new DigitalInput(2);
    }

    @Override
    public void periodic() {
        // Automatically prime when ShooterLazer is blocked
        if (isShooterLazerBlocked() && !isPulling) {
            pull(PULL_SPEED);
        } else if (!isShooterLazerBlocked() && isPulling) {
            stopPulling();
        }
    }

    public void pull(double speed) {
        shooterMotor.set(speed);
        isPulling = true;
    }

    public void stopPulling() {
        shooterMotor.set(0);
        isPulling = false;
    }

    public void shoot(double power) {
            if (!isShooting) {
                shooterMotor.set(power);
                isShooting = true;
            } else {
                shooterMotor.set(0);
                isShooting = false;
            };
    }

    public boolean isShooterLazerBlocked() {
        return !shooterLazer.get(); // False = blocked for break-beam
    }

    public boolean elevatorStopper() {
        return shooterLazer.get() && funnelLazer.get() && !exitLazer.get();
    }

    public boolean isPulling() {
        return isPulling;
    }
}