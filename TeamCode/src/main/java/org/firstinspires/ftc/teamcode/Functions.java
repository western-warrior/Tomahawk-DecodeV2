package org.firstinspires.ftc.teamcode;

public class Functions {
    public static boolean inBetween(int a, int max, int min) {
        return a < max && a > min;
    }

    public static boolean inBetween(double a, double max, double min) {
        return a < max && a > min;
    }

    public static boolean greaterThan(int a, int b) {
        return a > b;
    }

    public static boolean greaterThan(double a, double b) {
        return a > b;
    }

    public static boolean lessThan(int a, int b) {
        return a < b;
    }

    public static boolean lessThan(double a, double b) {
        return a < b;
    }
}
