// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Class to run the rollers over CAN */
public class ClimbSubsystem extends SubsystemBase {
  
  private final SparkMax rollerMotor;
  private ClimbSubsystem() {
    Arm.getInstance;
  }
  @Override
  public void periodic() {
  }
  /** This is a method that makes the roller spin */
  public void moveArm(double forward, double reverse) {
  }
}