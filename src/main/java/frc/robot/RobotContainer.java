// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Shooter.ShooterMode;

import java.util.function.BooleanSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController driverXbox = new CommandXboxController(
            ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorXbox = new CommandXboxController(
            ControllerConstants.OPERATOR_CONTROLLER_PORT);

    private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();

    private final UsbCamera intakeCam = CameraServer.startAutomaticCapture();

    // private final DriveCommand normalDrive = new DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

    private final Intake intake = new Intake();

    private final Shooter shooter = new Shooter();

    // ----------- Commands ---------- \\

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        // swerveDriveSubsystem.setDefaultCommand(normalDrive);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        driverXbox.a().whileTrue(new RepeatCommand(new InstantCommand(() -> {
            System.out.println(driverXbox.getRawAxis(0));
            intake.setCustomPercent(driverXbox.getRawAxis(0));
            shooter.setCustomPercent(driverXbox.getRawAxis(2) - driverXbox.getRawAxis(3));
        }))).onFalse(new InstantCommand(() -> {
            intake.setMode(IntakeMode.STOPPED);
            shooter.setMode(ShooterMode.STOPPED);
        }));


        driverXbox.x().and(new BooleanSupplier() {
            public boolean getAsBoolean() {
                return !intake.getFrontLimitClosed();
            }
        }).onTrue(
            new IntakeCommand(intake).raceWith(new WaitUntilCommand(driverXbox.x().negate()))
        );

        driverXbox.b().and(new BooleanSupplier() {
            public boolean getAsBoolean() {
                return intake.getReverseLimitClosed() || intake.getFrontLimitClosed();
            }
        }).onTrue(
            new ParallelRaceGroup(
                new WaitUntilCommand(driverXbox.b().negate()),
                new SequentialCommandGroup(
                    new AlignNoteCommand(intake, shooter),
                    new PrintCommand("EEEEEEEEEEEEEEEE"),
                    new PrepNoteCommand(shooter, intake),
                    new PrepShooterCommand(intake, shooter, 1.0),
                    new ShootCommand(shooter, intake)
                    // new InstantCommand(() -> {
                    //     shooter.coast();
                    //     shooter.setMode(ShooterMode.STOPPED);
                    // })
            ))
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // TODO: Implement Auto Command
        return null;
    }
}
