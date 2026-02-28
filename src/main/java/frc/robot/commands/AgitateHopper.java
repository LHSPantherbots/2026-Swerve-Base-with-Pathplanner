// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Hopper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AgitateHopper extends Command {
  IntakeRoller intakeRoller;
  IntakePivot intakePivot;
  Hopper hopper;
  Integer timer;
  Integer delay = 100;
  /** Creates a new AgitateHopper. */
  public AgitateHopper(IntakeRoller intakeRoller, IntakePivot intakePivot, Hopper hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeRoller = intakeRoller;
    this.intakePivot = intakePivot;
    this.hopper = hopper;
    addRequirements(intakeRoller, intakePivot, hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer = timer + 1;
    intakeRoller.intake();;
    hopper.forward();
    intakePivot.motionMagicSetPosition();
    Integer adjustedVal = timer/delay;

    if (adjustedVal % 2 == 0){
      System.out.println("mid");
      intakePivot.setIntakeMid();
    }else{
      System.out.println("down");
      intakePivot.setIntakeDown();;
    }
    
    //System.out.println("timer:" + timer);
    //System.out.println("adjustedVal" + adjustedVal);

    
    
    
    SmartDashboard.putNumber("Timer", timer);
    SmartDashboard.putNumber("Adjusted Value", adjustedVal );



    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakePivot.setIntakeDown();
    intakeRoller.stop();
    hopper.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
