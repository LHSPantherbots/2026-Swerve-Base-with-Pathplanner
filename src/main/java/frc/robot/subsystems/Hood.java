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

 
  private double allowableError = 0.1;
  //hood encoder is zeored when unhooked from the actuators and hood is lifted 
  //vertically until it hits hard stop of the encoder mount
  private double hoodMin = .79; // value read all the way down
  private double hoodMax = .86; // value read all the ay at actuator limits
  private double hoodSetpoint = hoodMin; //-pivot_zero_offset
  
  //Hood Position Setpoints
  private final double shortSetpoint = .79 ; //need to test emperically
  private final double midSetpoint = .81;
  private final double longSetpoint = .83;
  private final double extraLongSetpoint = .84;

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
                .inverted(true);

        c_hoodRight
                .apply(c_hoodGlobal);
             
                
        c_hoodRight.softLimit
            .forwardSoftLimit(hoodMax)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(hoodMin)
            .reverseSoftLimitEnabled(true);
                
                
        c_hoodRight.encoder
                
                .positionConversionFactor(1.0) // meters
                .velocityConversionFactor(1.0);//meters/sec
        c_EncoderConfig.zeroOffset(0);
                
                
//                  .inverted(false);
        c_hoodRight.closedLoop
                  .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                  .p(300.0)//this works, but need to go back and see if a smaller value would be better 300
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

  public void setHoodShort(){ //front of the hub 1meter- hood angle: 0.79  Launcher speed: 50
    setHoodSetpoint(shortSetpoint);
  }

    public void setHoodMid(){
    setHoodSetpoint(midSetpoint);
  }

    public void setHoodLong(){
    setHoodSetpoint(longSetpoint);
  }

  public void setHoodExtraLong(){
    setHoodSetpoint(extraLongSetpoint);
  }



}
