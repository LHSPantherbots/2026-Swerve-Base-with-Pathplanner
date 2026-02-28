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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AgitateHopper;
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
import frc.robot.subsystems.Leds;

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
    private final Leds leds = new Leds();


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
                ()->centerDrive.manualDrive(-m_driverController.getRightY()), centerDrive )
        );

        leds.setDefaultCommand(new RunCommand(()->leds.rainbow(), leds));

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

        /*DEFAULT CTRE CONTROLS

        m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        ));

        m_driverController.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        m_driverController.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        */

        // Reset the field-centric heading on start press.
         m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        //==================  DRIVER CONTROLLER ===============================

    // Right bumper: climb up at a fixed speed. Left bumper: climb down (reverse).
    m_driverController.rightBumper().whileTrue(new RunCommand(()->climb.manualDrive(0.75), climb));
    m_driverController.leftBumper().whileTrue(new RunCommand(()->climb.manualDrive(-0.75), climb));



        //=================   OPERATOR CONTROLLER ===============================
        m_operatorController.x().whileTrue( 
            new ConditionalCommand( 
                new RunCommand(() -> intakeRoller.eject(), intakeRoller), // if the left pumper is pressed while x is pressed it ejects
                new RunCommand(() -> intakeRoller.intake(), intakeRoller), // if only the x is pressed it intakes
                m_operatorController.leftBumper()::getAsBoolean ) );

     
        //operator must hold the left bumper and move the right joystick up or down to manually move the hood.  has a deadband to keep it from moving with stick drift
        m_operatorController.leftBumper().whileTrue(new RunCommand(()->hood.manualMove(MathUtil.applyDeadband(m_operatorController.getRightY(),.1)), hood));
        m_operatorController.leftBumper().onFalse(new InstantCommand(()->hood.setHoodSetpointToCurrentPosition(),hood));
        

        //Launcher Setpoints (D-Pad)
        m_operatorController.povDown().onTrue(new InstantCommand(()->hood.setHoodShort(), hood));
        m_operatorController.povDown().onTrue(new InstantCommand(()->launcher.setLauncherShort(), launcher));
        m_operatorController.povDown().onTrue(new RunCommand(()->leds.green(), leds));
        
        m_operatorController.povLeft().onTrue(new InstantCommand(()->hood.setHoodMid(), hood));
        m_operatorController.povLeft().onTrue(new InstantCommand(()->launcher.setLauncherMid(), launcher));
        m_operatorController.povLeft().onTrue(new RunCommand(()->leds.blue(), leds));

        m_operatorController.povUp().onTrue(new InstantCommand(()->hood.setHoodLong(), hood));
        m_operatorController.povUp().onTrue(new InstantCommand(()->launcher.setLauncherLong(), launcher));
        m_operatorController.povUp().onTrue(new RunCommand(()->leds.red(), leds));

        m_operatorController.povRight().onTrue(new InstantCommand(()->hood.setHoodExtraLong(), hood));
        m_operatorController.povRight().onTrue(new InstantCommand(()->launcher.setLauncherExtraLong(), launcher));
        m_operatorController.povDown().onTrue(new RunCommand(()->leds.purple(), leds));
        
        m_operatorController.leftTrigger().onTrue(new InstantCommand(()->launcher.setLauncherStop(), launcher));
        m_operatorController.leftTrigger().onTrue(new InstantCommand(()->hood.setHoodShort(),hood));
        m_operatorController.leftTrigger().onTrue(new RunCommand(()->leds.rainbow(), leds));



        m_operatorController.a().onTrue(new InstantCommand(()->intakePivot.setIntakeDown(), intakePivot));
        m_operatorController.b().onTrue(new InstantCommand(()->intakePivot.setIntakeMid(), intakePivot));
        m_operatorController.y().onTrue(new InstantCommand(()->intakePivot.setIntakeUp(), intakePivot));

        m_operatorController.rightTrigger().whileTrue(new AgitateHopper(intakeRoller, intakePivot, hopper, feeder));
        



        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
