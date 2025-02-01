package frc.robot.commands;
 
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.util.StringCoderReader;
import java.lang.Math;


public class ElevatorCommands {

    //Sets up variables
    private final Spark elevator_spark;
    private final StringCoderReader coder;
    private final double elevator_speed;
    private final double error;
    private final int elevator_spark_channel;
    double current_height;
    public ElevatorCommands(){
        elevator_spark_channel = ElevatorConstants.spark_channel;
        coder = new StringCoderReader(ElevatorConstants.coder_port, ElevatorConstants.voltage_to_distance_factor);
        elevator_spark = new Spark(elevator_spark_channel);
        current_height = coder.getDistance();
        elevator_speed = ElevatorConstants.elevator_speed;
        error = ElevatorConstants.error;
    }
    //Set Height Command moves motor up/down to the wanted height
    public void set_height(double wanted_height){
        double diff = current_height - wanted_height;
        while (Math.abs(diff) > error){
            if (diff<0){
                elevator_spark.set(elevator_speed);
            }
            else if (diff>0){
                elevator_spark.set(-elevator_speed);
            }  
        }
    }
    //This function returns the height of the elevator at that time
    public double get_height(){
        return current_height;
    }
}
