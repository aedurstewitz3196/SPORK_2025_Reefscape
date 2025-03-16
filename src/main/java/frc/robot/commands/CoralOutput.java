package frc.robot.commands;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralOutput extends Command {
    private final SparkMax shooterMotor;
    private final DigitalInput FunnelLazer;
    private final DigitalInput ShooterLazer;
    private final DigitalInput ExitLazer;
    boolean ispulling = false;
    boolean isshooting = false;

    public CoralOutput() {
        shooterMotor = new SparkMax(11, MotorType.kBrushless);
        FunnelLazer = new DigitalInput(0);
        ShooterLazer = new DigitalInput(1);
        ExitLazer = new DigitalInput(2);
    }

    public void pull() {
        while (!ShooterLazer.get() && !ispulling) {
            shooterMotor.set(.5);
            ispulling = true;
        }
        ispulling = false;
    }
    public void shoot(double power) {
        if (!isshooting){
            shooterMotor.set(power);
            isshooting = true;
        }
        isshooting = false;
    }
    public boolean elevatorstopper() {
        if (ShooterLazer.get() && FunnelLazer.get() && !ExitLazer.get()){
            return true;
        }
        return false;
    }

}
