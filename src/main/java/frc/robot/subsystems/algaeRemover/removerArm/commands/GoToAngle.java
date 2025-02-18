package frc.robot.subsystems.algaeRemover.removerArm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeRemover.removerArm.RemoverArm;
import frc.robot.subsystems.algaeRemover.removerArm.RemoverArm.RemoverArmControlType;

// controlled by pid 
public class GoToAngle extends Command {
    RemoverArm m_removerArm;
    double m_angle;

    public GoToAngle(RemoverArm removerArm, double angle) {
        m_removerArm = removerArm;
        m_angle = angle;

        addRequirements(removerArm);
    }

    public void initialize() {
        m_removerArm.setControlType(RemoverArmControlType.PID);
    }

    public void execute() {
        // makes arm go to angle set point
        m_removerArm.goToAngle(m_angle);
    }

    public boolean isFinished() {
        // confirmation that arm is at angle
        return m_removerArm.isAtAngle(); 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}