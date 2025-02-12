package frc.robot.subsystems.climb.arm.commands;

import frc.robot.Constants;
import frc.robot.subsystems.climb.arm.Arm;

// controlled by pid 
public class GoToGround extends GoToAngle {
    Arm m_arm;
    
    public GoToGround(Arm arm) {
        super(arm, Constants.ClimbConstants.kGroundPosition);
    }

}