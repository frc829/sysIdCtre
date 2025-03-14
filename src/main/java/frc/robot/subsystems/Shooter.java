package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;


public class Shooter extends SubsystemBase {
    // The motor on the shooter wheel .
    private final TalonFX motor = new TalonFX(14);
    private final CANcoder cancoder = new CANcoder(34);
    private final VoltageOut voltageOutFOC = new VoltageOut(0.0).withEnableFOC(true);

    // The shooter wheel encoder
    // Create a new SysId routine for characterizing the shooter.
    private final SysIdRoutine sysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(1.0),         // Use default ramp rate (1 V/s)
                            Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                            Seconds.of(10.0),          // Use default timeout (10 s)
                            // Log state with Phoenix SignalLogger class
                            state -> SignalLogger.writeString("state", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            volts -> motor.setControl(voltageOutFOC.withOutput(volts)),
                            null,
                            this
                    )
            );

    /**
     * Creates a new Shooter subsystem.
     */
    public Shooter() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.getConfigurator().apply(config);
        BaseStatusSignal.setUpdateFrequencyForAll(250,
                motor.getRotorPosition(),
                motor.getRotorVelocity(),
                motor.getMotorVoltage(),
                motor.getPosition(),
                motor.getVelocity(),
                cancoder.getAbsolutePosition(),
                cancoder.getVelocity());
        motor.optimizeBusUtilization();
        SignalLogger.start();
    }


    public Command sysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
