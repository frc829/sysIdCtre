// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SysIdRoutineBot {
    // The robot's subsystems
    private final Shooter shooter = new Shooter();

    // The driver's controller
    CommandXboxController driverController = new CommandXboxController(0);

    /**
     * Use this method to define bindings between conditions and commands. These are useful for
     * automating robot behaviors based on button and sensor input.
     *
     * <p>Should be called in the robot class constructor.
     *
     * <p>Event binding methods are available on the {@link Trigger} class.
     */
    public void configureBindings() {
        // Control the drive with split-stick arcade controls


        // Bind full set of SysId routine tests to buttons; a complete routine should run each of these
        // once.
        // Using bumpers as a modifier and combining it with the buttons so that we can have both sets
        // of bindings at once


        // Control the shooter wheel with the left trigger

        driverController
                .a()
                .whileTrue(shooter.sysIdQuasistaticForward())
                .onFalse(shooter.createStop());
        driverController
                .b()
                .whileTrue(shooter.sysIdQuasistaticReverse())
                .onFalse(shooter.createStop());
        driverController
                .x()
                .whileTrue(shooter.sysIdDynamicForward())
                .onFalse(shooter.createStop());
        driverController
                .y()
                .whileTrue(shooter.sysIdDynamicReverse())
                .onFalse(shooter.createStop());
    }

}
