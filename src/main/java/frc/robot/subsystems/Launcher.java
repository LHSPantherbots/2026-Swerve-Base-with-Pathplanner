// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.CanIdConstants;




public class Launcher extends SubsystemBase {
    
  private final TalonFX leftLauncher;
  private final TalonFX rightLauncher;
  private double launcherSetpoint = 0; //rotations per second

  //Setpoints   need to emperically figure these out
  private double shortSetpoint = 30;
  private double midSetpint = 40;
  private double longSetpoint = 50;

  /* Be able to switch which control request to use based on a button press */
  /* Start at velocity 0, use slot 0 */
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  /* Start at velocity 0, use slot 1 */
  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);
  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();
  /** Creates a new Launcher. */
  public Launcher() {

    leftLauncher = new TalonFX(CanIdConstants.launcherLeftCanId);
    rightLauncher = new TalonFX(CanIdConstants.launcherRightCanId);

        TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(8)
      .withPeakReverseVoltage(-8);

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(40)
      .withPeakReverseTorqueCurrent(-40);

    /* Retry config apply up to 5 times, report if failure */
    StatusCode statusr = StatusCode.StatusCodeNotInitialized;
    StatusCode statusl = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusr = rightLauncher.getConfigurator().apply(configs);
      if (statusr.isOK()) break;
    }
        for (int i = 0; i < 5; ++i) {
      statusl = leftLauncher.getConfigurator().apply(configs);
      if (statusl.isOK()) break;
    }
    
    if (!statusr.isOK()) {
      System.out.println("Could not apply configs, error code: " + statusr.toString());
    }
    if (!statusl.isOK()) {
      System.out.println("Could not apply configs, error code: " + statusl.toString());
    }

   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getNumber("Launcher Setpoint Rotations per Second", launcherSetpoint);
    SmartDashboard.getNumber("Right Launcher Speed Rotations per second", rightLauncher.getVelocity().getValueAsDouble());
    SmartDashboard.getNumber("Left Launcher Speed Rotations per second", leftLauncher.getVelocity().getValueAsDouble());
    SmartDashboard.getNumber(" Right Launcher Voltage", rightLauncher.getMotorVoltage().getValueAsDouble());
    SmartDashboard.getNumber("Left Launcher Voltage", leftLauncher.getMotorVoltage().getValueAsDouble());
    SmartDashboard.getNumber("Right Launcher Amp", rightLauncher.getStatorCurrent().getValueAsDouble());
    SmartDashboard.getNumber("Left Launcher Amp", leftLauncher.getStatorCurrent().getValueAsDouble());

  }

  public void closedLoopVelocityLaunchVoltage() {
    
      leftLauncher.setControl(m_velocityVoltage.withVelocity(launcherSetpoint));
      rightLauncher.setControl(m_velocityVoltage.withVelocity(launcherSetpoint));

  }


  public void closedLoopVelocityTorqueLaunch(){
    leftLauncher.setControl(m_velocityTorque.withVelocity(launcherSetpoint));
    rightLauncher.setControl(m_velocityTorque.withVelocity(launcherSetpoint));

  }


  public void setLauncherSetpoint(double value){
    launcherSetpoint = value;
  }

  public double getLauncherSetpoint(){
    return launcherSetpoint;
  }

  public void setLauncherShort(){
    setLauncherSetpoint(shortSetpoint);
  }

  public void setlauncherMid(){
    setLauncherSetpoint(midSetpint);
  }

  public void setlauncherLong(){
    setLauncherSetpoint(longSetpoint);
  }

  
  

  
}
