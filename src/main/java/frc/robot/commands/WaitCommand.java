package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitCommand extends Command {
    Timer WaitTime;
    double TimeToWait;
    public WaitCommand(double time){
        TimeToWait=time;
        WaitTime = new Timer();
        WaitTime.start();
    
    }
    @Override
    public boolean isFinished(){
        return WaitTime.hasElapsed(TimeToWait);
            
        
    }

}
