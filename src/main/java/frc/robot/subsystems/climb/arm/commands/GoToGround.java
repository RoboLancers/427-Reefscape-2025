package frc.robot.subsystems.Climb.arm.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Climb.arm.Arm;

// controlled by pid 
public class GoToGround extends GoToAngle {
    Arm m_arm;
    
    public GoToGround(Arm arm) {
        super(arm, Constants.ClimbConstants.kGroundPosition);
    }

}