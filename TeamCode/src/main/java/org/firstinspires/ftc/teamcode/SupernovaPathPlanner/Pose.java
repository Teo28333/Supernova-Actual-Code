package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

/**
 * Documentation ajoutee: Objet mathematique pour position x/y et cap, avec operations de distance, rotation et normalisation.
 */
public class Pose {
    private double x;
    private double y;
    private double heading;

    /**
     * Create a pose in field coordinates.
     *
     * @param x x coordinate in inches
     * @param y y coordinate in inches
     * @param heading heading in radians
     *
     */
    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    // Construit cette classe avec les dependances et reglages necessaires.
    public Pose() {
        this(0.0, 0.0, 0.0);
    }

    // Construit cette classe avec les dependances et reglages necessaires.
    public Pose(Pose pose) {
        this(pose.x, pose.y, pose.heading);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getX() {
        return x;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setX(double x) {
        this.x = x;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getY() {
        return y;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setY(double y) {
        this.y = y;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getHeading() {
        return heading;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setHeading(double heading) {
        this.heading = heading;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getHeadingDegrees() {
        return Math.toDegrees(heading);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setHeadingDegrees(double headingDegrees) {
        this.heading = Math.toRadians(headingDegrees);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void set(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void set(Pose pose) {
        set(pose.x, pose.y, pose.heading);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public Pose copy() {
        return new Pose(x, y, heading);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public Pose withX(double x) {
        return new Pose(x, y, heading);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public Pose withY(double y) {
        return new Pose(x, y, heading);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public Pose withHeading(double heading) {
        return new Pose(x, y, heading);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public Pose plus(Pose pose) {
        return new Pose(x + pose.x, y + pose.y, heading + pose.heading);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public Pose minus(Pose pose) {
        return new Pose(x - pose.x, y - pose.y, heading - pose.heading);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public Pose times(double scalar) {
        return new Pose(x * scalar, y * scalar, heading * scalar);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public Pose div(double scalar) {
        if (scalar == 0.0) {
            throw new IllegalArgumentException("Cannot divide a pose by zero.");
        }

        return new Pose(x / scalar, y / scalar, heading / scalar);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public double distanceTo(Pose pose) {
        return Math.hypot(pose.x - x, pose.y - y);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public double headingTo(Pose pose) {
        return Math.atan2(pose.y - y, pose.x - x);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public double headingErrorTo(Pose pose) {
        return normalizeRadians(pose.heading - heading);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public Pose rotated(double angleRadians) {
        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);

        return new Pose(
                (x * cos) - (y * sin),
                (x * sin) + (y * cos),
                heading + angleRadians
        );
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public static Pose fromDegrees(double x, double y, double headingDegrees) {
        return new Pose(x, y, Math.toRadians(headingDegrees));
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public static Pose zero() {
        return new Pose();
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public static double normalizeRadians(double angleRadians) {
        double normalized = Math.IEEEremainder(angleRadians, 2.0 * Math.PI);

        if (normalized <= -Math.PI) {
            normalized += 2.0 * Math.PI;
        }

        if (normalized > Math.PI) {
            normalized -= 2.0 * Math.PI;
        }

        return normalized;
    }
}
