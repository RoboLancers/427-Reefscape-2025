// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.RollerConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Degrees;

import java.io.Console;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ClimbSubsystem extends SubsystemBase {
  //Create here :)
  private DigitalInput climbBeamBreak;
  private boolean climbBeamBreakValue;
  private double targetPosition = 0;
  private SparkMax climbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushed);
  private PIDController climbPIDController = new PIDController(Constants.ClimbConstants.kP, Constants.ClimbConstants.kI, Constants.ClimbConstants.kD);
  private SparkAbsoluteEncoder encoder = climbMotor.getAbsoluteEncoder();
  private ArmFeedforward feedforward = new ArmFeedforward(Constants.ClimbConstants.kS, Constants.ClimbConstants.kG, Constants.ClimbConstants.kV);


  public ClimbSubsystem() {
    setupSpark();
  }
  //Configures motors and encoders
  public void setupSpark() {
    SparkMaxConfig config = new SparkMaxConfig(); 
    
    config.idleMode(IdleMode.kBrake);

    config.inverted(Constants.ClimbConstants.kLeftMotorInverted);
    
    config.smartCurrentLimit(Constants.ClimbConstants.kMotorCurrentLimit);
    
    config.encoder.positionConversionFactor(Constants.ClimbConstants.kAbsPositionConversionFactor);
    config.encoder.velocityConversionFactor(Constants.ClimbConstants.kAbsVelocityConversionFactor);
    config.encoder.positionConversionFactor(Constants.ClimbConstants.kRelativePositionConversionFactor); 
    config.encoder.velocityConversionFactor(Constants.ClimbConstants.kRelativeVelocityConversionFactor); 

    climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  @Override
  public void periodic() {
    // Gets the desired speed to get to the target position from the current position. 
    // Feedforward is making sure the arm doesn't fall or move too far back.
   double velocity = climbPIDController.calculate(getAngle(), targetPosition) + feedforward.calculate(targetPosition, 0);
   climbMotor.set(velocity);
 

  }
  // Goes to the starting position.
  public void goToInitial(){
    goToAngle(ClimbConstants.deployPosition);
}
  // Goes to the desired position to climb.
public void goToClimb(){
    goToAngle(ClimbConstants.climbPosition);
}
  // Changes the target position from the starting postion to the desired position.
public void goToAngle(Angle angle){
  this.targetPosition = angle.in(Degrees);
}
 // Gets the value of the beambreak
  public boolean getclimbBeambreakvalue() {
    return climbBeamBreak.get();
  } 
  // Gets the current position.
  public double getAngle() {
    return encoder.getPosition();
  }
}