// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CenterDrive;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakePivot;

import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Launcher;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    private final CenterDrive centerDrive = new CenterDrive();
    private final IntakeRoller intakeRoller = new IntakeRoller();
    private final IntakePivot intakePivot = new IntakePivot();


    private final Hopper hopper = new Hopper();
    private final Feeder feeder = new Feeder();
    private final Hood hood = new Hood();
    private final Climb climb = new Climb();
    private final Launcher launcher = new Launcher();


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Left Stick Controls
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                     .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                     .withRotationalRate((m_driverController.getLeftTriggerAxis()-m_driverController.getRightTriggerAxis()) * MaxAngularRate*2.0) // Drive counterclockwise with negative X (left)
            )
        );

        // TODO: make this a run command under diverconroller
        //Right stick y drives centerdrive
        // has a deadband to keep it from moving with stick drift

        //Noah Right stick
        centerDrive.setDefaultCommand(
            new RunCommand(
                ()->centerDrive.manualDrive(MathUtil.applyDeadband(m_driverController.getRightY(), 0.09)), centerDrive ));
        // center wheel stop when not in use
        intakeRoller.setDefaultCommand(
            new RunCommand(
                ()->intakeRoller.manualDrive(0.0), intakeRoller) //defualts with roller not spinning
        );

        
        intakePivot.setDefaultCommand(
            new RunCommand(
                //()->hood.stop(), hood )
                ()->intakePivot.motionMagicSetPosition(), intakePivot)
        );

        hopper.setDefaultCommand(
            new RunCommand(
                ()->hopper.manualDrive(0.0), hopper) //defualts with roller not spinning
        );

        feeder.setDefaultCommand(
            new RunCommand(
                ()->feeder.manualDrive(0.0), feeder) //defualts with roller not spinning
        );

        hood.setDefaultCommand(
            new RunCommand(
                //()->hood.stop(), hood )
                ()->hood.closedLoopHood(), hood)
        );

        launcher.setDefaultCommand(
            new RunCommand(
                ()->launcher.closedLoopVelocityLaunchVoltage(),launcher)
        );
        // sets climb speed to 0 when not in use
        // Climb: run while driver right bumper is held (boolean). Uses the
        // Climb.manualDrive(boolean) convenience method added to the subsystem.(

    climb.setDefaultCommand(
       new RunCommand(
           ()->climb.manualDrive(0.0), climb )
       );

        

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );


        //==================  DRIVER CONTROLLER ===============================

    // Right bumper: climb up at a fixed speed. Left bumper: climb down (reverse).
    m_driverController.rightBumper().whileTrue(new RunCommand(()->climb.manualDrive(0.75), climb));
    m_driverController.leftBumper().whileTrue(new RunCommand(()->climb.manualDrive(-0.75), climb));



        //=================   OPERATOR CONTROLLER ===============================

        //operator must hold the X button to run the intake roller at the default speed for intaking.  when released, it will stop the roller.
        m_operatorController.x().whileTrue(new RunCommand(()->intakeRoller.intake(), intakeRoller));

        //operator must hold the right trigger to run the hopper forward and the feeder forward at the default speeds for shooting.  when released, they will stop.
        m_operatorController.rightTrigger().whileTrue(new RunCommand(()->hopper.forward(), hopper).alongWith(new RunCommand(()->feeder.forward(), feeder)));

        //operator must hold the left bumper and move the left joystick up or down to manually move the hood.  has a deadband to keep it from moving with stick drift
        m_operatorController.leftBumper().whileTrue(new RunCommand(()-> hood.manualMove(MathUtil.applyDeadband(m_operatorController.getLeftY(),.1)), hood));

        //operator must click the left trigger to stop the launcher.
        m_operatorController.leftTrigger().onTrue(new RunCommand(() -> launcher.stopLauncher(), launcher));

        //operator must hold the left bumper and x to run the intake roller in reverse at the default speed for ejecting.  when released, it will stop the roller.
        m_operatorController.leftBumper().and(m_operatorController.x()).whileTrue(new RunCommand(() -> intakeRoller.reverseIntake(), intakeRoller));

        //operator must hold the left bumper and move the right joystick up or down to manually move the intakepivot.  has a deadband to keep it from moving with stick drift
        m_operatorController.leftBumper().whileTrue(new RunCommand(()-> intakePivot.manualDrive(MathUtil.applyDeadband(m_operatorController.getRightY(),.1)), intakePivot));



        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
