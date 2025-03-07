// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeRemover;

import frc.robot.subsystems.algaeRemover.removerArm.RemoverArm;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.AlgaeRemoverConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Class to run the rollers over CAN */
public class AlgaeRemoverSubsystem extends SubsystemBase {
  
  private final SparkMax removerMotor;
  private AlgaeRemoverSubsystem() {
    RemoverArm.getInstance();
    removerMotor = new SparkMax(AlgaeRemoverConstants.REMOVER_MOTOR_ID, MotorType.kBrushed);
  }
  @Override
  public void periodic() {
  }
  /** This is a method that makes the roller spin */
  public void moveRemoverArm(double forward, double reverse) {
  }
}