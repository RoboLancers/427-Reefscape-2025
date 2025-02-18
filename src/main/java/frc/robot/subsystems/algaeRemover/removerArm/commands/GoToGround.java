package frc.robot.subsystems.algaeRemover.removerArm.commands;

import frc.robot.Constants;
import frc.robot.subsystems.algaeRemover.removerArm.RemoverArm;

// controlled by pid 
public class GoToGround extends GoToAngle {
    RemoverArm m_arm;
    
    public GoToGround(RemoverArm removerArm) {
        super(removerArm, Constants.AlgaeRemoverConstants.kGroundPosition);
    }

}