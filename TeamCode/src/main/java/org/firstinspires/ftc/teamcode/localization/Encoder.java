package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class Encoder {
    public static final double FORWARD = 1.0;
    public static final double REVERSE = -1.0;

    private Encoder() {
    }

    public static GoBildaPinpointDriver.EncoderDirection toPinpointDirection(double direction) {
        return direction < 0.0
                ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                : GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}
