package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * Subsystem for controlling LED lights through a CTRE CANdle.
 * 
 * Uses a priority system to decide what pattern to show:
 *   Climbing (highest) > Shooting > Enabled > Disabled (lowest)
 * 
 * Other subsystems set flags (like setClimbing(true)) and this
 * subsystem picks the highest-priority active state every loop.
 */
public class LEDs extends SubsystemBase {

    // ---- LED State Enum ----
    // Lists every possible LED state in priority order (lowest to highest)
    public enum LEDState {
        DISABLED,   // Robot is disabled — solid orange
        ENABLED,    // Robot is enabled — solid teal
        SHOOTING,   // Spindexer/shooter running — chase teal
        CLIMBING    // Climbing — chase orange
    }

    // The CANdle hardware device — ID 7 on the CANivore bus
    private final CANdle candle = new CANdle(7, Constants.kCANBus);

    // LED index range: 0-7 onboard + 8-167 strip = 168 total
    private static final int FIRST_LED = 0;
    private static final int LAST_LED = 167;

    // ---- Colors ----
    private static final RGBWColor TEAL = new RGBWColor(0, 142, 131, 0);     // #008E83
    private static final RGBWColor ORANGE = new RGBWColor(255, 138, 61, 0);  // #FF8A3D

    // ---- Control Requests (created once, reused each loop) ----

    // Solid color requests
    private final SolidColor solidTeal = new SolidColor(FIRST_LED, LAST_LED)
        .withColor(TEAL);

    private final SolidColor solidOrange = new SolidColor(FIRST_LED, LAST_LED)
        .withColor(ORANGE);

    // Chase animation requests
    private final ColorFlowAnimation chaseTeal = new ColorFlowAnimation(FIRST_LED, LAST_LED)
        .withColor(TEAL)
        .withSlot(0)
        .withFrameRate(25)
        .withDirection(AnimationDirectionValue.Forward);

    private final ColorFlowAnimation chaseOrange = new ColorFlowAnimation(FIRST_LED, LAST_LED)
        .withColor(ORANGE)
        .withSlot(0)
        .withFrameRate(25)
        .withDirection(AnimationDirectionValue.Forward);

    // ---- State Tracking ----
    // These flags are set by other parts of the code to tell the LEDs what's happening
    private boolean isShooting = false;
    private boolean isClimbing = false;

    // Tracks what state we showed last loop, so we only send a new command
    // when something actually changes (avoids flooding the CAN bus)
    private LEDState lastState = null;

    public LEDs() {
        // Configure the CANdle's settings
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = 0.75;
        candle.getConfigurator().apply(config);
    }

    /**
     * Called every 20ms by the command scheduler automatically.
     * This is where we decide which LED pattern to show based
     * on the current robot state and activity flags.
     */
    @Override
    public void periodic() {
        // Determine the current state using our priority system
        LEDState currentState = getCurrentState();

        // Only send a new control if the state actually changed
        // (like only changing the TV channel when you pick a new one)
        if (currentState != lastState) {
            applyState(currentState);
            lastState = currentState;
        }
    }

    /**
     * Figures out what LED state we should be in right now.
     * Checks from highest priority to lowest — the first match wins.
     */
    private LEDState getCurrentState() {
        // Highest priority: climbing
        if (isClimbing) {
            return LEDState.CLIMBING;
        }

        // Next: shooting/spindexer
        if (isShooting) {
            return LEDState.SHOOTING;
        }

        // Base states: enabled vs disabled
        if (DriverStation.isDisabled()) {
            return LEDState.DISABLED;
        } else {
            return LEDState.ENABLED;
        }
    }

    /**
     * Sends the appropriate control request to the CANdle
     * based on the desired state.
     */
    private void applyState(LEDState state) {
        switch (state) {
            case DISABLED:
                candle.setControl(solidOrange);
                break;
            case ENABLED:
                candle.setControl(solidTeal);
                break;
            case SHOOTING:
                candle.setControl(chaseTeal);
                break;
            case CLIMBING:
                candle.setControl(chaseOrange);
                break;
        }
    }

    // ---- Public Methods for Other Subsystems to Call ----

    /** Call this with true when the spindexer/shooter starts, false when it stops */
    public void setShooting(boolean shooting) {
        this.isShooting = shooting;
    }

    /** Call this with true when climbing starts, false when it stops */
    public void setClimbing(boolean climbing) {
        this.isClimbing = climbing;
    }
}