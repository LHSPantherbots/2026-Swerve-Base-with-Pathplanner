package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Hood;

public class IntakeCmd extends Command {
    IntakeRoller intake;
    Feeder feeder;
    Hood hood;
    boolean noteWasDetected = false;
    boolean shouldEnd = false;
    

    public IntakeCmd(
            IntakeRoller intake, Feeder feeder) {
        this.intake = intake;
        this.feeder = feeder;
        addRequirements(intake, feeder);
    }

    @Override
    public void initialize() {
        this.intake.intake();
        this.feeder.forward();
    }

    @Override
    public void execute() {

    }
    //possibly delete this ^

    @Override
    public void end(boolean interrupted) {
        this.intake.intakeStop();
        this.shouldEnd = false;
    }

    @Override
    public boolean isFinished() {
        return this.shouldEnd;
    }

}