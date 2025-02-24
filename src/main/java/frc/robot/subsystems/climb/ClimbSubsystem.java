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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.measure.Distance;

/** Class to run the rollers over CAN */
public class ClimbSubsystem extends SubsystemBase {
  //Create here :)
  private DigitalInput climbBeamBreak;
  private boolean climbBeamBreakValue;
  private double targetPosition = 0;
  private climbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushed);
  private PIDController m_armPIDController = new PIDController(Constants.ClimbConstants.kP, Constants.ClimbConstants.kI, Constants.ClimbConstants.kD);



  public ClimbSubsystem() {
    public void setupSpark() {
      SparkMaxConfig config = new SparkMaxConfig();
  ;
      config.idleMode(IdleMode.kBrake);
  
      config.inverted(Constants.ClimbConstants.kLeftMotorInverted);
      
      config.smartCurrentLimit(Constants.ClimbConstants.kMotorCurrentLimit);
      
      // right arm motor would follow left arm motor's voltage 
      config.encoder.positionConversionFactor(Constants.ClimbConstants.kAbsPositionConversionFactor);
      config.encoder.velocityConversionFactor(Constants.ClimbConstants.kAbsVelocityConversionFactor);
      config.encoder.positionConversionFactor(Constants.ClimbConstants.kRelativePositionConversionFactor); 
      config.encoder.velocityConversionFactor(Constants.ClimbConstants.kRelativeVelocityConversionFactor); 
  
  }
  
  public void setupControllers() {
    // Traposition error on which it is tolerable
   // m_armPIDController.setTolerance(Constants.ClimbConstants.kTolerance);
  }
  }
  public void goToDeploy(){
      goToAngle(ClimbConstants.deployPosition);
  }
  public void goToClimb(){
      goToAngle(ClimbConstants.climbPosition);
  }
  public void goToAngle(Angle angle){
    this.targetPosition = angle.in(Degrees);
  }

  
  @Override
  public void periodic() {
    //The value of the climb beambreak.
    climbBeambreakvalue = getclimbBeambreakvalue();
    // If the climb is engaged with the cage, then the motor will stop running. Might change the value the motor is set to later.
    if(climbBeambreakvalue==true){
      ClimbMotor.set(0,0);
    }
    
    // Gets the value of the climb beambreak.
   
  }
  
  public boolean getclimbBeambreakvalue() {
    return climbBeambreak.get();
  } 
}