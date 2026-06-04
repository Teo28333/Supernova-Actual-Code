package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/**
 * Documentation ajoutee: Utilitaires pour representer le sens des encodeurs et convertir vers les constantes Pinpoint.
 */
public class Encoder {
    public static final double FORWARD = 1.0;
    public static final double REVERSE = -1.0;

    // Construit cette classe avec les dependances et reglages necessaires.
    private Encoder() {
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public static GoBildaPinpointDriver.EncoderDirection toPinpointDirection(double direction) {
        return direction < 0.0
                ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                : GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}
