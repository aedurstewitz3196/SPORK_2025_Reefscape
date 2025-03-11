package frc.robot.commands;
 
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.util.StringCoderReader;
import java.lang.Math;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ElevatorCommands {

    //Sets up variables
    private final SparkMax elevator_spark_nine;
    //private final SparkMax elevator_spark_ten;
    private final StringCoderReader coder;
    private final double elevator_speed;
    private final double error;
    private final int elevator_spark_channel;
    double current_height;

    public ElevatorCommands(){
        elevator_spark_channel = ElevatorConstants.spark_channel;
        coder = new StringCoderReader(ElevatorConstants.coder_port, ElevatorConstants.voltage_to_distance_factor);
        elevator_spark_nine = new SparkMax(elevator_spark_channel, MotorType.kBrushless);
        //elevator_spark_ten = new SparkMax(10,MotorType.kBrushless);
        current_height = coder.getDistance();
        elevator_speed = ElevatorConstants.elevator_speed;
        error = ElevatorConstants.error;
    }
    //Set Height Command moves motor up/down to the wanted height
    public void set_height(double wanted_height, boolean stopper){
        if (true){
            double diff = current_height - wanted_height;
            while (Math.abs(diff) > error){
                diff = (coder.getDistance() - wanted_height);
                if (diff<0){
                    //System.out.println("Elevator moving up");
                    elevator_spark_nine.set(elevator_speed);
                    //elevator_spark_ten.set(-elevator_speed);
                }
                else if (diff>0){
                    //System.out.println("Elevator moving down");
                    elevator_spark_nine.set(-elevator_speed);
                    //elevator_spark_ten.set(elevator_speed);
                }  
            }
        }
        else {
            System.out.println("Coral being pulled. Please wait");
        }
    }
    //This function returns the height of the elevator at that time
    public double get_height(){
        return current_height;
    }
}
