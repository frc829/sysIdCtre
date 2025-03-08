package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
        // The motor on the shooter wheel .
        private final SparkMax motor = new SparkMax(17, MotorType.kBrushless);

        // The shooter wheel encoder
        // Create a new SysId routine for characterizing the shooter.
        private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
                        new SysIdRoutine.Config(
                                        null, // Use default ramp rate (1 V/s)
                                        Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                                        null, // Use default timeout (10 s)
                                        // Log state with Phoenix SignalLogger class
                                        state -> SignalLogger.writeString("wrist", state.toString())),
                        new SysIdRoutine.Mechanism(
                                        volts -> {
                                                motor.setVoltage(volts.baseUnitMagnitude());
                                                SignalLogger.writeDouble("Voltage",
                                                                motor.getAppliedOutput() * motor.getBusVoltage());
                                        },
                                        null,
                                        this));

        /**
         * Creates a new Shooter subsystem.
         */
        public Shooter() {
                TalonFXConfiguration config = new TalonFXConfiguration();
                motor.getConfigurator().apply(config);
                BaseStatusSignal.setUpdateFrequencyForAll(250,
                                motor.getPosition(),
                                motor.getVelocity(),
                                motor.getMotorVoltage());
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
