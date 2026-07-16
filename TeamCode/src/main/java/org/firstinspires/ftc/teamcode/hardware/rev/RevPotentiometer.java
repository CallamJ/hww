package org.firstinspires.ftc.teamcode.hardware.rev;

import org.firstinspires.ftc.teamcode.hardware.SmartAnalogInput;
import org.firstinspires.ftc.teamcode.hardware.SmartPotentiometer;

/**
 * A class that converts an analog signal into a potentiometer based absolute encoder.
 * This class allows reading angular positions and adjusting offsets.
 */
public class RevPotentiometer extends SmartPotentiometer {

    /**
     * Constructs a `RevPotentiometer` instance with the given parameters.
     *
     * @param input      The `AnalogInput` representing the potentiometer's analog sensor.
     * @param name       The unique name used for persistent storage of the offset value.
     * @param maxAngle   The maximum angle (in degrees) the potentiometer can measure.
     * @param maxVoltage The maximum voltage output of the potentiometer at its highest position.
     */
    public RevPotentiometer(SmartAnalogInput input, String name, double maxAngle, double maxVoltage) {
        super(input, name, maxAngle, maxVoltage);
    }

    /**
     * Constructs a `RevPotentiometer` instance with the given parameters.
     *
     * @param input      The `AnalogInput` representing the potentiometer's analog sensor.
     * @param name       The unique name used for persistent storage of the offset value.
     * @param maxAngle   The maximum angle (in degrees) the potentiometer can measure.
     * @param maxVoltage The maximum voltage output of the potentiometer at its highest position.
     * @param offset
     */
    public RevPotentiometer(SmartAnalogInput input, String name, double maxAngle, double maxVoltage, double offset) {
        super(input, name, maxAngle, maxVoltage, offset);
    }

    @Override
    protected double voltageToAngle(double v){
        return -1.96682 * Math.pow(v, 3) - 7.04864 * Math.pow(v, 2) + 126.39282 * v;
    }
}
