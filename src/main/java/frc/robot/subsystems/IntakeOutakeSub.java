package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeOutakeSub extends SubsystemBase {
    public PIDController intakeArmPidController = new PIDController(IntakeConstants.kArmP, IntakeConstants.kArmI, IntakeConstants.kArmD);

    private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
    
    private CANSparkMax leftIntakeArmMotor = new CANSparkMax(IntakeConstants.kLeftIntakeArmMotorID, MotorType.kBrushless);
    private CANSparkMax rightIntakeArmMotor = new CANSparkMax(IntakeConstants.kRightIntakeArmMotorID, MotorType.kBrushless);

    private DutyCycleEncoder intakeArmEncoder = new DutyCycleEncoder(IntakeConstants.kIntakeArmEncoderPort);
    private double debugintakeArmSetpoint = 0;
    public void IntakeSub(){
    }

    public void setIntakeSpeed(boolean intakeBool, boolean outakeBool){
        if(intakeBool){
            intakeMotor.set(IntakeConstants.kIntakeSpeed);
        
        }else if (outakeBool){
            intakeMotor.set(-IntakeConstants.kIntakeSpeed);

        } else {
            intakeMotor.set(0);
        }
    } // 
    public void setIntakeArmMotorSpeed0 (){
        rightIntakeArmMotor.set(0);
    }
    public void setIntakeArmMotorSetpoint (double intakeArmSetpoint){

        debugintakeArmSetpoint = intakeArmSetpoint;
        intakeArmPidController.setSetpoint(intakeArmSetpoint);
        // sets the setpoint in the PID  Controller


        double intakeArmSpeed = intakeArmPidController.calculate(intakeArmEncoder.getAbsolutePosition());
        // converts it into speeds
        if (intakeArmSpeed > IntakeConstants.kIntakeArmMaxSpeed){
        rightIntakeArmMotor.set(IntakeConstants.kIntakeArmMaxSpeed);
        leftIntakeArmMotor.set(-IntakeConstants.kIntakeArmMaxSpeed);
        } else {
            rightIntakeArmMotor.set(intakeArmSpeed);
            leftIntakeArmMotor.set(intakeArmSpeed);

        }
    }

    @Override
    public void periodic(){
        /// telmetry

        SmartDashboard.putNumber("IntakeArmPos", intakeArmEncoder.get());
        SmartDashboard.putNumber("IntakeArmAbsPos", intakeArmEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("IntakeArmPower", rightIntakeArmMotor.getAppliedOutput());
        SmartDashboard.putNumber("IntakeDesired", intakeArmPidController.getSetpoint());


        //Debug Setpoint
    }
    public void resetEncoders(){
        intakeArmEncoder.reset();
    }

    

}