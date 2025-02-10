package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;


public class Shooter extends SubsystemBase {
    // The motor on the shooter wheel .
    private final SparkMax motor = new SparkMax(17, SparkLowLevel.MotorType.kBrushless);
    // private final SparkMax follower = new SparkMax(25, SparkLowLevel.MotorType.kBrushless);


    // The shooter wheel encoder

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle position = Radian.mutable(0);
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
                                        .angularPosition(position.mut_setMagnitude(motor.getEncoder().getPosition()))
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
        double factor = 2 * Math.PI / 70.0 / (32.0/18.0);
        EncoderConfig encoderConfig = new EncoderConfig()
                .uvwMeasurementPeriod(16)
                .uvwAverageDepth(2)
                .positionConversionFactor(factor)
                .velocityConversionFactor(factor / 60.0);
        SparkBaseConfig config = new SparkFlexConfig()
                .apply(encoderConfig);
        // SparkBaseConfig followerConfig = new SparkFlexConfig()
        //         .follow(15, true);
        config.idleMode(SparkBaseConfig.IdleMode.kCoast);
        config.inverted(false);
        // followerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        // follower.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
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
