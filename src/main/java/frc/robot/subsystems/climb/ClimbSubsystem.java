// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.climb.arm.Arm;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.RollerConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

/** Class to run the rollers over CAN */
public class ClimbSubsystem extends SubsystemBase {
  
  private final Arm climbArm;
  private final SparkMax climbMotor;
  private final AbsoluteEncoder climbEncoder;
  private final PIDController climbPID;
  SparkMaxConfig climbConfig;

  private ClimbSubsystem() {
    climbArm = Arm.getInstance();
    climbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushed);
    climbEncoder = climbMotor.getAbsoluteEncoder();
    climbPID = new PIDController(Constants.ClimbConstants.kP, Constants.ClimbConstants.kI, Constants.ClimbConstants.kD);

    configMotor();
    configEncoder();
    configController();
  }

  private void configMotor() {
    this.climbConfig = new SparkMaxConfig();
    this.climbConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
    this.climbConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
    this.climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configEncoder() {

  }

  private void configController() {
  }

  @Override
  public void periodic() {
    //Tthe value of the climb beambreak.
    this.climbBeambreakvalue = getclimbBeambreakvalue();
    // If the climb is engaged with the cage, then the motor will stop running. Might change the value the motor is set to later.
    if(climbBeambreakvalue==true){
      ClimbMotor.set(0,0);
    }
    // Gets the value of the climb beambreak.

    public boolean getclimbBeambreakvalue() {
      return this.climbBeambreak.get();
    }

  /** This is a method that makes the roller spin */
  public void moveArm(double forward, double reverse) {
  }
}