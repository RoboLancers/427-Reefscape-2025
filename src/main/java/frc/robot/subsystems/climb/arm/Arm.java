package frc.robot.subsystems.Climb.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
public class Arm extends SubsystemBase {
    
    private static Arm instance = new Arm(); 

    public static Arm getInstance() {
        return instance; 
    }
    
    // Define and initiate variables for arm
    private double m_targetPosition = Constants.ClimbConstants.kTravelPosition;
    private double m_manualVelocity = 0;

    private DigitalInput m_limitSwitch = new DigitalInput(Constants.ClimbConstants.kLimitSwitchId);

    private SparkMax m_armMotorLeft = new SparkMax(Constants.ClimbConstants.kArmMotorLeftId, MotorType.kBrushless);

    // encoder from the right motor
    private AbsoluteEncoder m_armAbsoluteEncoder = m_armMotorLeft.getAbsoluteEncoder();
    private RelativeEncoder m_armRelativeEncoder = m_armMotorLeft.getEncoder();
    
    private PIDController m_armPIDController = new PIDController(Constants.ClimbConstants.kP, Constants.ClimbConstants.kI, Constants.ClimbConstants.kD);
    
    public ArmFeedforward m_armFeedforward = new ArmFeedforward(Constants.ClimbConstants.kS, Constants.ClimbConstants.kG, Constants.ClimbConstants.kV, Constants.ClimbConstants.kA);

    private ArmControlType m_ArmControlType = Arm.ArmControlType.PID;


    private Arm() {
        setupControllers();
        setupSpark();
    }

    // motor config
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

    //encoder config
    

    // pid controller config
    public void setupControllers() {
        // position error on which it is tolerable
        m_armPIDController.setTolerance(Constants.ClimbConstants.kTolerance);
    }

    public void periodic() {
        doSendables();
        
        double impendingVelocity = 0; 

        if (m_ArmControlType == ArmControlType.PID) {

            impendingVelocity = m_armPIDController.calculate(getAngle(), m_targetPosition) 
                                + m_armFeedforward.calculate(Math.toRadians(getAngle()), 0);
        }
        
        // velocity controlled manually
        else if (m_ArmControlType == ArmControlType.MANUAL) {
            impendingVelocity = m_manualVelocity;
        }

        // if the arm is not reaching beyond the limit switch,
        // or if the arm is not reaching beyond 100 degs, 
        // arm is allowed to move 
        // forward defined as away from the limit switch.
        boolean passReverseSoftLimit = reverseSoftLimit() && impendingVelocity < 0;
        boolean passForwardSoftLimit = forwardSoftLimit() && impendingVelocity > 0;

        // if (!passReverseSoftLimit && !passForwardSoftLimit) {
            m_armMotorLeft.set(impendingVelocity); 
        // }

        SmartDashboard.putNumber("Impending Velocity (m/s)", impendingVelocity);
        SmartDashboard.putBoolean("Pass Reverse Soft Limit", passReverseSoftLimit);
        SmartDashboard.putBoolean("Pass Forward Soft Limit", passForwardSoftLimit);
    }

    public boolean reverseSoftLimit() {
        // return (getLimitSwitchValue() || getAngle() < Constants.ClimbConstants.kReverseSoftLimit);
        return (getAngle() < Constants.ClimbConstants.kReverseSoftLimit);
    }

    public boolean forwardSoftLimit() {
        return getAngle() > Constants.ClimbConstants.kForwardSoftLimit;
    }

    public boolean getLimitSwitchValue() {
        return m_limitSwitch.get(); 
    }

    // public void setKG(double kG) {
    //     this.m_kG = kG;
    // }

    // public void setKS(double kS) {
    //     this.m_kS = kS;
    // }

    public double getError() {
        return m_armPIDController.getPositionError();
    }

    public void goToAngle(double angle) {
        this.m_targetPosition = angle;
    }

    public double getAngle() {
        return Math.toDegrees(MathUtil.angleModulus(Math.toRadians(m_armAbsoluteEncoder.getPosition())));
    }


    public void setSpeed(double speed) {
        this.m_manualVelocity = speed;
    }

    public boolean isAtAngle() {
        return m_armPIDController.atSetpoint();
        
    }

    public void setControlType(ArmControlType type) {
        this.m_ArmControlType = type;
    }

    public void setPID(double p, double i, double d) {
        this.m_armPIDController.setPID(p, i, d);
    }

    public ArmControlState getArmControlState() {
        if (m_targetPosition == Constants.ClimbConstants.kGroundPosition) {
            return ArmControlState.GROUND;
        }
        else if (m_targetPosition == Constants.ClimbConstants.kTravelPosition) {
            return ArmControlState.TRAVEL;
        }
        
        else {
            return ArmControlState.CUSTOM;
        }
    }

    public enum ArmControlType {
        MANUAL, 
        PID
    }

    public enum ArmControlState {
        GROUND,
        TRAVEL,
        AMP,
        SPEAKER, 
        CUSTOM
    }

    // add logging for arm 
    // are units correct?
    public void doSendables() {
        SmartDashboard.putNumber("Arm Target Position (deg)", m_targetPosition);
        SmartDashboard.putNumber("Arm Position (deg)", getAngle()); 
        SmartDashboard.putNumber("Arm Absolute Position (deg)", m_armAbsoluteEncoder.getPosition()); 
        SmartDashboard.putNumber("Arm Velocity (deg/sec)", m_armAbsoluteEncoder.getVelocity());
        SmartDashboard.putNumber("Arm Error (deg)", getError());
        SmartDashboard.putBoolean("Arm At Set Point", isAtAngle());
        SmartDashboard.putString("Arm Control Type", m_ArmControlType.toString());
        SmartDashboard.putString("Arm Control State", getArmControlState().toString());
        SmartDashboard.putNumber("left volt", m_armMotorLeft.get()); 

        SmartDashboard.putBoolean("Arm Limit Switch", getLimitSwitchValue());
        // SmartDashboard.putBoolean("left inverted", m_armMotorLeft.getInverted()); 
        // SmartDashboard.putBoolean("right inverted", m_armMotorRight.getInverted()); 
    }
}