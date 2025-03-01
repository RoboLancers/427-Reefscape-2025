package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitCommand extends Command {
    //Creates Wait time and time to wait variables
    Timer WaitTime;
    double TimeToWait;
    public WaitCommand(double time){
        //Creates a variable for time to wait and defins it as time
        TimeToWait=time;
        //Creates a new timer
        WaitTime = new Timer();
        // Starts the timer
        WaitTime.start();
    
    }
    @Override
    public boolean isFinished(){
        //Checks weather the time to wait has passed. 
        return WaitTime.hasElapsed(TimeToWait);
            
        
    }

}
