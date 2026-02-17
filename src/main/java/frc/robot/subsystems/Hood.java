// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.CanIdConstants;

public class Hood extends SubsystemBase {

  private  SparkMax hoodLeft;
  private  SparkMax hoodRight;
  private  SparkMaxConfig c_hoodGlobal = new SparkMaxConfig(); 
  private  SparkMaxConfig c_hoodRight = new SparkMaxConfig();
  private  SparkMaxConfig c_hoodLeft = new SparkMaxConfig();
  AbsoluteEncoder e_hoodEncoder;
  private AbsoluteEncoderConfig c_EncoderConfig = new AbsoluteEncoderConfig();
  private final SparkClosedLoopController hoodController;

  private double pivot_zero_offset = .1078;  //wrist was zeroed vertically up 
                                            //agains the elevator for initial setpoints then was take above the top of the elevator and 
                                            //moved past the initial zero by this amount then rezeroed.
  private double hoodSetpoint = 0.75; //-pivot_zero_offset
  private double allowableError = 0.1;

  private ShuffleboardTab tab = Shuffleboard.getTab("Tuning");  //angles used for shuffleboard; taken from 2024 fulcrum code
  private GenericEntry sbAngle = tab.add("Hood Angle", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 90))
            .getEntry();
  
            
  public Hood() {
        
        hoodRight = new SparkMax(CanIdConstants.hoodRightCanId, MotorType.kBrushed);
        hoodLeft = new SparkMax(CanIdConstants.hoodLeftCanId, MotorType.kBrushed);
        hoodController = hoodRight.getClosedLoopController();
        e_hoodEncoder = hoodRight.getAbsoluteEncoder();

       


        c_hoodGlobal
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(5)
                .inverted(false);

        c_hoodRight
                .apply(c_hoodGlobal);
             
                
        c_hoodRight.softLimit
            .forwardSoftLimit(0.9-pivot_zero_offset)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(0.56)
            .reverseSoftLimitEnabled(true);
                
                
        c_hoodRight.encoder
                
                .positionConversionFactor(1.0) // meters
                .velocityConversionFactor(1.0);//meters/sec
        c_EncoderConfig.zeroOffset(0);
                
                
//                  .inverted(false);
        c_hoodRight.closedLoop
                  .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                  .p(3.0)//need to tune
                  .i(0)
                  .d(0.0)
                  .maxOutput(1.00)
                  .minOutput(-1.0);

        c_hoodLeft
                  .apply(c_hoodGlobal)
                  .follow(hoodRight)

                    ;
        
        hoodRight.configure(c_hoodRight, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        hoodLeft.configure(c_hoodLeft, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        
        

  }

  @Override
  public void periodic() {

      SmartDashboard.putNumber("Hood Left Output", hoodLeft.getAppliedOutput());
      SmartDashboard.putNumber("Hood Setpoint", hoodSetpoint);
      SmartDashboard.putNumber("Hood Pos", e_hoodEncoder.getPosition());
  
  }


  public void stop() {
        hoodRight.set(0.0);
  }

  public void manualMove(double move) {
        hoodRight.set(move);
  }

  public void setHoodSetpoint(double setpoint){
        hoodSetpoint = setpoint;
  }

  public double getHoodSetPoint(){
        return hoodSetpoint;
  }

  public void setHoodSetpointToCurrentPosition(){
      setHoodSetpoint(e_hoodEncoder.getPosition());
    }

  public void closedLoopHood() {
        // m_FulcrumRight.set(m_Controller.calculate(e_FulcrumEncoder.getPosition(),
        // setPoint));
        

        double sp = getHoodSetPoint();
        hoodController.setSetpoint(sp, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }


    public boolean isAtAngle() {
        double error = e_hoodEncoder.getPosition() - hoodSetpoint;
        return (Math.abs(error) < allowableError);
      }

}
