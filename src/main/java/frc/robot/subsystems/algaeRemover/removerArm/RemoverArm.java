package frc.robot.subsystems.algaeRemover.removerArm;

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
 
public class RemoverArm extends SubsystemBase {
    
    private static RemoverArm instance = new RemoverArm(); 

    public static RemoverArm getInstance() {
        return instance; 
    }
    
    // Define and initiate variables for arm
    private double m_targetPosition = Constants.AlgaeRemoverConstants.kTravelPosition;
    private double m_manualVelocity = 0;

    private DigitalInput m_limitSwitch = new DigitalInput(Constants.AlgaeRemoverConstants.kLimitSwitchId);

    private SparkMax m_removerArmMotor = new SparkMax(Constants.AlgaeRemoverConstants.kRemoverArmMotorId, MotorType.kBrushless);

    // encoder from the right motor
    private AbsoluteEncoder m_removerArmAbsoluteEncoder = m_removerArmMotor.getAbsoluteEncoder();
    private RelativeEncoder m_removerArmRelativeEncoder = m_removerArmMotor.getEncoder();
    
    private PIDController m_removerArmPIDController = new PIDController(Constants.AlgaeRemoverConstants.kP, Constants.AlgaeRemoverConstants.kI, Constants.AlgaeRemoverConstants.kD);
    
    public ArmFeedforward m_removerArmFeedforward = new ArmFeedforward(Constants.AlgaeRemoverConstants.kS, Constants.AlgaeRemoverConstants.kG, Constants.AlgaeRemoverConstants.kV, Constants.AlgaeRemoverConstants.kA);

    private RemoverArmControlType m_removerRemoverArmControlType = RemoverArm.RemoverArmControlType.PID;


    private RemoverArm() {
        setupControllers();
        setupSpark();
    }

    // motor config
    public void setupSpark() {
        SparkMaxConfig config = new SparkMaxConfig();
;
        config.idleMode(IdleMode.kBrake);

        config.inverted(Constants.AlgaeRemoverConstants.kRemoverMotorInverted);
        
        config.smartCurrentLimit(Constants.AlgaeRemoverConstants.kMotorCurrentLimit);
        
        // right arm motor would follow left arm motor's voltage 
        config.encoder.positionConversionFactor(Constants.AlgaeRemoverConstants.kAbsPositionConversionFactor);
        config.encoder.velocityConversionFactor(Constants.AlgaeRemoverConstants.kAbsVelocityConversionFactor);
        config.encoder.positionConversionFactor(Constants.AlgaeRemoverConstants.kRelativePositionConversionFactor); 
        config.encoder.velocityConversionFactor(Constants.AlgaeRemoverConstants.kRelativeVelocityConversionFactor); 
        
    }

    //encoder config
    

    // pid controller config
    public void setupControllers() {
        // position error on which it is tolerable
        m_removerArmPIDController.setTolerance(Constants.AlgaeRemoverConstants.kTolerance);
    }

    public void periodic() {
        doSendables();
        
        double impendingVelocity = 0; 

        if (m_removerRemoverArmControlType == RemoverArmControlType.PID) {

            impendingVelocity = m_removerArmPIDController.calculate(getAngle(), m_targetPosition) 
                                + m_removerArmFeedforward.calculate(Math.toRadians(getAngle()), 0);
        }
        
        // velocity controlled manually
        else if (m_removerRemoverArmControlType == RemoverArmControlType.MANUAL) {
            impendingVelocity = m_manualVelocity;
        }

        // if the arm is not reaching beyond the limit switch,
        // or if the arm is not reaching beyond 100 degs, 
        // arm is allowed to move 
        // forward defined as away from the limit switch.
        boolean passReverseSoftLimit = reverseSoftLimit() && impendingVelocity < 0;
        boolean passForwardSoftLimit = forwardSoftLimit() && impendingVelocity > 0;

        // if (!passReverseSoftLimit && !passForwardSoftLimit) {
            m_removerArmMotor.set(impendingVelocity); 
        // }

        SmartDashboard.putNumber("Impending Velocity (m/s)", impendingVelocity);
        SmartDashboard.putBoolean("Pass Reverse Soft Limit", passReverseSoftLimit);
        SmartDashboard.putBoolean("Pass Forward Soft Limit", passForwardSoftLimit);
    }

    public boolean reverseSoftLimit() {
        // return (getLimitSwitchValue() || getAngle() < Constants.AlgaeRemoverConstants.kReverseSoftLimit);
        return (getAngle() < Constants.AlgaeRemoverConstants.kReverseSoftLimit);
    }

    public boolean forwardSoftLimit() {
        return getAngle() > Constants.AlgaeRemoverConstants.kForwardSoftLimit;
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
        return m_removerArmPIDController.getPositionError();
    }

    public void goToAngle(double angle) {
        this.m_targetPosition = angle;
    }

    public double getAngle() {
        return Math.toDegrees(MathUtil.angleModulus(Math.toRadians(m_removerArmAbsoluteEncoder.getPosition())));
    }


    public void setSpeed(double speed) {
        this.m_manualVelocity = speed;
    }

    public boolean isAtAngle() {
        return m_removerArmPIDController.atSetpoint();
        
    }

    public void setControlType(RemoverArmControlType type) {
        this.m_removerRemoverArmControlType = type;
    }

    public void setPID(double p, double i, double d) {
        this.m_removerArmPIDController.setPID(p, i, d);
    }

    public ArmControlState getArmControlState() {
        if (m_targetPosition == Constants.AlgaeRemoverConstants.kGroundPosition) {
            return ArmControlState.GROUND;
        }
        else if (m_targetPosition == Constants.AlgaeRemoverConstants.kTravelPosition) {
            return ArmControlState.TRAVEL;
        }
        
        else {
            return ArmControlState.CUSTOM;
        }
    }

    public enum RemoverArmControlType {
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
        SmartDashboard.putNumber("Arm Absolute Position (deg)", m_removerArmAbsoluteEncoder.getPosition()); 
        SmartDashboard.putNumber("Arm Velocity (deg/sec)", m_removerArmAbsoluteEncoder.getVelocity());
        SmartDashboard.putNumber("Arm Error (deg)", getError());
        SmartDashboard.putBoolean("Arm At Set Point", isAtAngle());
        SmartDashboard.putString("Arm Control Type", m_removerRemoverArmControlType.toString());
        SmartDashboard.putString("Arm Control State", getArmControlState().toString());
        SmartDashboard.putNumber("left volt", m_removerArmMotor.get()); 

        SmartDashboard.putBoolean("Arm Limit Switch", getLimitSwitchValue());
        // SmartDashboard.putBoolean("left inverted", m_removerArmMotor.getInverted()); 
        // SmartDashboard.putBoolean("right inverted", m_armMotorRight.getInverted()); 
    }
}