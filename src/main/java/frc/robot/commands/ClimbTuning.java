// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.climb.ClimbSubsystem;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

  // Command to run the roller with joystick inputs
public class ClimbTuning extends Command {
  
  // private final CANRollerSubsystem rollerSubsystem;
  private final ClimbSubsystem climb;
  

  public ClimbTuning(ClimbSubsystem climb) {
    this.climb = climb;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
   // Creates variable for kP, kI, and kD, and gets the value of them from the Smartdashboard.
   double kP = SmartDashboard.getNumber("kP", 0);
   double kI = SmartDashboard.getNumber("kI", 0);
   double kD = SmartDashboard.getNumber("kD", 0);

   // Changes the values of P,I, and D,
   climb.changePID(kP, kI, kD);
   // Gets the value of the angle from the SmartDashBoard.It gets the position in degrees.
   double angle = SmartDashboard.getNumber("angle(deg)", Constants.ClimbConstants.deployPosition.in(Degrees));
    

   climb.goToAngle(Degrees.of(angle));
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
