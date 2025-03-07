// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import java.util.function.DoubleSupplier;

// Command to run the roller with joystick inputs
public class GoToInitial extends Command {
  
  // private final CANRollerSubsystem rollerSubsystem;
  private final ClimbSubsystem climb;
  

  public GoToInitial(ClimbSubsystem climb) {
    this.climb = climb;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    //Makes the climb go to the starting position
    climb.goToInitial();
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {

  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    //Checks to see if the position the climb is at is the desired position.
    return climb.isAtAngle();
  }
}
