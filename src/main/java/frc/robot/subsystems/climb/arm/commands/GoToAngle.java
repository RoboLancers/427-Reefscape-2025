package frc.robot.subsystems.Climb.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.arm.Arm;
import frc.robot.subsystems.Climb.arm.Arm.ArmControlType;

// controlled by pid 
public class GoToAngle extends Command {
    Arm m_arm;
    double m_angle;

    public GoToAngle(Arm arm, double angle) {
        m_arm = arm;
        m_angle = angle;

        addRequirements(arm);
    }

    public void initialize() {
        m_arm.setControlType(ArmControlType.PID);
    }

    public void execute() {
        // makes arm go to angle set point
        m_arm.goToAngle(m_angle);
    }

    public boolean isFinished() {
        // confirmation that arm is at angle
        return m_arm.isAtAngle(); 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}