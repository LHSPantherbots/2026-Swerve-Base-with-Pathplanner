package frc.robot.subsystems;


import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Climb extends SubsystemBase{

     // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.

    private  SparkMax m_Climb; 
    private  SparkMaxConfig c_Climb = new SparkMaxConfig();
    private RelativeEncoder c_Encoder;    
    

    
    // setting what moter is set
    public Climb() {      
        m_Climb = new SparkMax(13, MotorType.kBrushless);
        c_Encoder = m_Climb.getEncoder();


        c_Climb
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(37)
                
                .inverted(false); // TODO: should be able to removed
       
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
  // stop command
     public void stop() {
        m_Climb.set(0.0);
    }

     public double getPosition() {
        return c_Encoder.getPosition();
    }

    
    // makeing a value to use on our divercontroller bumpers
    // TODO: see if this is even needed and what can be removed
    public void upclimb(double value) {
        m_Climb.set(value);
    }
    
    public void manualDrive(double speed) {
        upclimb(speed);
    }

    public void manualDrive(boolean enabled) {
        manualDrive(enabled ? 0.75 : 0.0);
    }



    
 }