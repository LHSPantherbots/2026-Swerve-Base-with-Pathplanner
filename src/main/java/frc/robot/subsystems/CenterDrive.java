// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.CanIdConstants;

public class CenterDrive extends SubsystemBase {
  
  private final TalonFX centerDrive;
  private final DutyCycleOut talonOut = new DutyCycleOut(0);


  public CenterDrive() {
    centerDrive = new TalonFX(CanIdConstants.centerDriveCanId);



    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 60; //Output Current Limit
   
    cfg.MotionMagic.MotionMagicCruiseVelocity = 50; // 5 rotations per second
    cfg.MotionMagic.MotionMagicAcceleration = 100; // Take approximately 0.5 seconds to reach max vel
    cfg.MotionMagic.MotionMagicJerk = 1000;// Take approximately 0.2 seconds to reach max accel 
    cfg.TorqueCurrent.PeakForwardTorqueCurrent = 50; //Current Limit value used in FOC Torque Mode
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = 50; //Current Limit value used in FOC Torque Mode
  }

  //@Override
  //public void periodic() {
    // This method will be called once per scheduler run
  //}



  public void manualDrive(double value){
    talonOut.Output = value;
    talonOut.EnableFOC = false;
    centerDrive.setControl(talonOut);

  }
}
