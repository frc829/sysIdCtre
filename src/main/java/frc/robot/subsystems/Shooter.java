package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;


public class Shooter extends SubsystemBase {
    // The motor on the shooter wheel .
    private final SparkMax motor = new SparkMax(16, SparkLowLevel.MotorType.kBrushless);


    // The shooter wheel encoder

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle angle = Radians.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

    // Create a new SysId routine for characterizing the shooter.
    private final SysIdRoutine sysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,         // Use default ramp rate (1 V/s)
                            Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                            null          // Use default timeout (10 s)
                    ),
                    new SysIdRoutine.Mechanism(
                            volts -> motor.setVoltage(volts.baseUnitMagnitude()),
                            log -> {
                                // Record a frame for the shooter motor.
                                log.motor("motor")
                                        .voltage(
                                                appliedVoltage.mut_setMagnitude(
                                                        motor.getBusVoltage() * motor.getAppliedOutput()))
                                        .angularPosition(angle.mut_setMagnitude(motor.getEncoder().getPosition()))
                                        .angularVelocity(
                                                velocity.mut_setMagnitude(motor.getEncoder().getVelocity()));
                            },
                            this
                    )
            );

    /**
     * Creates a new Shooter subsystem.
     */
    public Shooter() {

        EncoderConfig encoderConfig = new EncoderConfig()
                .uvwMeasurementPeriod(16)
                .uvwAverageDepth(2)
                .positionConversionFactor(Math.PI * 2 / 12.0)
                .velocityConversionFactor(Math.PI * 2 / 60.0 / 12.0);
        SparkBaseConfig config = new SparkMaxConfig()
                .apply(encoderConfig);
        config.idleMode(SparkBaseConfig.IdleMode.kCoast);
        config.inverted(false);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }

    public void stop() {
        motor.stopMotor();
    }

    public Command createStop() {
        return runOnce(this::stop);
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
