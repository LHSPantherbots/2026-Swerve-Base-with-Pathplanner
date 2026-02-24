package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.CanIdConstants;

public class Launcher extends SubsystemBase {

  private final TalonFX leftLauncher;
  private final TalonFX rightLauncher;

  private double launcherSetpoint = 0;

  private double shortSetpoint = 50;
  private double midSetpoint = 55;
  private double longSetpoint = 60;
  private double extraLongSetpoint = 70;

  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);
  private final NeutralOut m_brake = new NeutralOut();

  public Launcher() {

    leftLauncher = new TalonFX(CanIdConstants.launcherLeftCanId);
    rightLauncher = new TalonFX(CanIdConstants.launcherRightCanId);

    // -----------------------------
    // Base config
    // -----------------------------
    TalonFXConfiguration base = new TalonFXConfiguration();

    // Slot 0 (voltage velocity)
    base.Slot0.kS = 0.1;
    base.Slot0.kV = 0.12;
    base.Slot0.kP = 0.11;
    base.Slot0.kI = 0;
    base.Slot0.kD = 0;

    base.Voltage.withPeakForwardVoltage(11)
                .withPeakReverseVoltage(-11);

    // Slot 1 (torque velocity)
    base.Slot1.kS = 2.5;
    base.Slot1.kP = 5;
    base.Slot1.kI = 0;
    base.Slot1.kD = 0;

    base.TorqueCurrent.withPeakForwardTorqueCurrent(60)
                      .withPeakReverseTorqueCurrent(-60);

    // -----------------------------
    // Left config
    // -----------------------------
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    leftConfig = base.clone();  // Phoenix 6 supports clone()
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // -----------------------------
    // Right config
    // -----------------------------
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    rightConfig = base.clone();
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // -----------------------------
    // Apply configs
    // -----------------------------
    StatusCode statusL = leftLauncher.getConfigurator().apply(leftConfig);
    StatusCode statusR = rightLauncher.getConfigurator().apply(rightConfig);

    if (!statusL.isOK()) System.out.println("Left config failed: " + statusL);
    if (!statusR.isOK()) System.out.println("Right config failed: " + statusR);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Launcher Setpoint RPS", launcherSetpoint);
    SmartDashboard.putNumber("Right Launcher Speed RPS", rightLauncher.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Left Launcher Speed RPS", leftLauncher.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right Launcher Voltage", rightLauncher.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Left Launcher Voltage", leftLauncher.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Right Launcher Amps", rightLauncher.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Left Launcher Amps", leftLauncher.getStatorCurrent().getValueAsDouble());
  }

  public void closedLoopVelocityLaunchVoltage() {
    leftLauncher.setControl(m_velocityVoltage.withVelocity(launcherSetpoint));
    rightLauncher.setControl(m_velocityVoltage.withVelocity(launcherSetpoint));
  }

  public void closedLoopVelocityTorqueLaunch() {
    leftLauncher.setControl(m_velocityTorque.withVelocity(launcherSetpoint));
    rightLauncher.setControl(m_velocityTorque.withVelocity(launcherSetpoint));
  }

  public void setLauncherSetpoint(double value) {
    launcherSetpoint = value;
  }

  public double getLauncherSetpoint() {
    return launcherSetpoint;
  }

  public void setLauncherStop() {
    setLauncherSetpoint(0);
  }

  public void setLauncherShort() { //front of the hub 1meter- hood angle: 0.79  Launcher speed: 50
    setLauncherSetpoint(shortSetpoint);
  }

  public void setLauncherMid() {
    setLauncherSetpoint(midSetpoint);
  }

  public void setLauncherLong() {
    setLauncherSetpoint(longSetpoint);
  }

    public void setLauncherExtraLong() {
    setLauncherSetpoint(extraLongSetpoint);
  }
  
}
