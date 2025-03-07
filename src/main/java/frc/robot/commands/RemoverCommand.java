// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeRemover.AlgaeRemoverSubsystem;

import java.util.function.DoubleSupplier;

public class RemoverCommand extends Command {
    private final DoubleSupplier forward;
    private final DoubleSupplier reverse;
    private final AlgaeRemoverSubsystem algaeRemoverSubsystem;

    public RemoverCommand (DoubleSupplier forward, DoubleSupplier reverse, AlgaeRemoverSubsystem removerSubsystem) {
        this.forward = forward;
        this.reverse = reverse;
        this.algaeRemoverSubsystem = removerSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        algaeRemoverSubsystem.moveRemoverArm(forward.getAsDouble(), reverse.getAsDouble());
    }

    @Override
    public void end(boolean isInterrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
