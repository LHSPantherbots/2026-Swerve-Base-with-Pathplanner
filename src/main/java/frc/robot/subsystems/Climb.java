package frc.robot.subsystems;


import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;



public class Climb extends SubsystemBase{

     // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.



    private  SparkMax m_Climb; 
    private  SparkMaxConfig c_Climb = new SparkMaxConfig();
    private RelativeEncoder c_Encoder;    
    

    

    public Climb() {      
        m_Climb = new SparkMax(13, MotorType.kBrushless);
        c_Encoder = m_Climb.getEncoder();


        c_Climb
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                
                .inverted(false);
        //c_Climb.softLimit
           // .forwardSoftLimit(30.0)
           // .forwardSoftLimitEnabled(true)
            //.reverseSoftLimit(-300)//230
            //.reverseSoftLimitEnabled(true);
                
                
        c_Climb.encoder
                
                .positionConversionFactor(1.0) // meters
                .velocityConversionFactor(1.0);//meters/sec
 
        //m_Climb.configure(c_Climb, ResetMode.kResetSafeParameters,
                    //PersistMode.kPersistParameters);
        //c_Encoder.setPosition(0.0);
        
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", c_Encoder.getPosition());
    
    }

     public void stop() {
        m_Climb.set(0.0);
    }

     public double getPosition() {
        return c_Encoder.getPosition();
    }

    public void upclimb(double value) {
        m_Climb.set(value);
    }
    
    // New overload: allow direct speed control (positive = up, negative = down)
    public void manualDrive(double speed) {
        upclimb(speed);
    }

    // Backwards-compatible method: previous callers that passed a boolean will
    // keep the same behavior (true -> move up at 0.5, false -> stop)
    public void manualDrive(boolean enabled) {
        manualDrive(enabled ? 0.5 : 0.0);
    }



    
 }