// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import java.util.function.DoubleSupplier;

// Command to run the roller with joystick inputs
public class ClimbCommand extends Command {
  private final double angle;
  private final boolean deploy;
  // private final CANRollerSubsystem rollerSubsystem;
  private final ClimbSubsystem climber;

  public ClimbCommand(
      double angle, boolean deploy, ClimbSubsystem rollerSubsystem) {
    this.angle = angle;
    this.deploy = deploy;
    this.climber = rollerSubsystem;

    addRequirements(this.rollerSubsystem);
  }

  @Override
  public void initialize() {
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    if (deploy==true){
    // deployPosition.get(;
    }
    
    climber.arm.goToAngle(this.angle);
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // Return false to indicate that this command never ends. It can be interrupted
    // by another command needing the same subsystem.
    return false;
  }
}
