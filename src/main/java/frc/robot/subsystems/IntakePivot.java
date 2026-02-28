// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.CanIdConstants;

public class IntakePivot extends SubsystemBase {
  
  private final TalonFX intakePivot;
  private final CANcoder intakePivotEncoder;
  
  private final DutyCycleOut talonOut = new DutyCycleOut(0);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private double intakePivotSetpoint = .172;
  private double allowableError = 1.0;

  private final double intakeGearRatio = 72.0/20.0 * 5.0 * 4.0;

  private double intakePivotDownLimit = .172; //-.147;///Update before use
  private double intakePivotUpLimit = .004;


  public IntakePivot() {
    intakePivot = new TalonFX(CanIdConstants.intakePivotCanId);
    intakePivotEncoder = new CANcoder(CanIdConstants.intakePivotEncoderCanId);


    //need to make zero at the top or bottom
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    //cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cc_cfg.MagnetSensor.withMagnetOffset(Rotations.of(.32));
    intakePivotEncoder.getConfigurator().apply(cc_cfg);

    TalonFXConfiguration cfg = new TalonFXConfiguration();



   
    cfg.Feedback.FeedbackRemoteSensorID = intakePivotEncoder.getDeviceID();
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.SensorToMechanismRatio = 1.0;
    cfg.Feedback.RotorToSensorRatio = intakeGearRatio;



    



    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    //cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 70.9;
    //cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    //cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 60; //Output Current Limit
    //cfg.CurrentLimits.SupplyTimeThreshold = 5; //Amont of time to allow current over supply limit
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 60; //Supply Current Limit
    cfg.MotionMagic.MotionMagicCruiseVelocity = 50; //50; // 5 rotations per second cruise
    cfg.MotionMagic.MotionMagicAcceleration = 100; //100; // Take approximately 0.5 seconds to reach max vel
    cfg.MotionMagic.MotionMagicJerk = 1000; //1000;// Take approximately 0.2 seconds to reach max accel 
    cfg.TorqueCurrent.PeakForwardTorqueCurrent = 50; //Current Limit value used in FOC Torque Mode
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = 50; //Current Limit value used in FOC Torque Mode

    m_mmReq.EnableFOC = false;



    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 300/12.8;
    slot0.kI = 0;
    slot0.kD = 0.1;
    slot0.kV =  0.12;
    slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving
    slot0.kG = -0.25; // 3V / 12V = 0.25 //tune upward until mechanism holds 

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 1.0;

    

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      status = intakePivot.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    
    intakePivot.setPosition(0);




  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Pivot Motor Position", intakePivot.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake Pivot Setpoint", intakePivotSetpoint);
    SmartDashboard.putNumber("Intake Pivot Motor Velocity ",intakePivot.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake Pivot Encoder Position", intakePivotEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake Pivot Encoder Abs Position", intakePivotEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake Pivot Supply Current",intakePivot.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake Pivot Stator Current",intakePivot.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake Pivot Motor Voltage", intakePivot.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Intake Pivot Supply Voltage", intakePivot.getSupplyVoltage().getValueAsDouble());
  
  }

  
  public void manualDrive(double value){
    talonOut.Output = value;
    talonOut.EnableFOC = true;
    intakePivot.setControl(talonOut);

  }
  public void motionMagicSetPosition(){
    intakePivot.setControl(m_mmReq.withPosition(intakePivotSetpoint).withSlot(0));
  }


  public void setZero(){
    intakePivot.setPosition(0.0);
  }

  public double getPosition(){
    double pos = intakePivot.getPosition().getValueAsDouble();
    return pos;
  }

  public void setIntakeSetpoint(double pos){
    intakePivotSetpoint = pos;
  }

  public boolean isAtHeight() {
    double error = getPosition() - intakePivotSetpoint;
    return (Math.abs(error) < allowableError);
  }

  public void setIntakeDown(){
    setIntakeSetpoint(intakePivotDownLimit);
  }

  public void setIntakeUp(){
    setIntakeSetpoint(intakePivotUpLimit);
  }

  public void setIntakeMid(){
    setIntakeSetpoint(0.04);
  }



}
