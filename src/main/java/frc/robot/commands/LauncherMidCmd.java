package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class LauncherMidCmd extends Command {
    Launcher launcher;

    public LauncherMidCmd(Launcher launcher) {
        this.launcher = launcher;
        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        this.launcher.setLauncherShort();
    }

    @Override
    public void end(boolean interrupted) {
        this.launcher.setLauncherStop();
    }
}