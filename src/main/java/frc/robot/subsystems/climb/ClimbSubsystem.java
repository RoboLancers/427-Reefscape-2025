// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import frc.robot.subsystems.climb.arm.Arm;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ClimbConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Class to run the rollers over CAN */
public class ClimbSubsystem extends SubsystemBase {
  
  private final SparkMax climbMotor;
  public final  Arm climbarm;
  private ClimbSubsystem() {
    this.climbarm = Arm.getInstance();
    this.climbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushed);
  }
  public void goToDeploy(){
    climbarm.goToAngle(ClimbConstants.deployPosition);
  }
  public void goToClimb(){
    climbarm.goToAngle(ClimbConstants.climbPosition);
  }
  @Override
  public void periodic() {
    //Tthe value of the climb beambreak.
    //this.climbBeambreakvalue = getclimbBeambreakvalue();
    // If the climb is engaged with the cage, then the motor will stop running. Might change the value the motor is set to later.
    //if(climbBeambreakvalue==true){
    //  ClimbMotor.set(0,0);
    //}
    
    // Gets the value of the climb beambreak.
   
  /** This is a method that makes the roller spin */
  }
  
 // public boolean getclimbBeambreakvalue() {
 //   return this.climbBeambreak.get();
  //} 
}