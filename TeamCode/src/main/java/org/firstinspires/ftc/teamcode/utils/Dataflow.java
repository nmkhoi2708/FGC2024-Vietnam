package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Dataflow {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    Telemetry telemetry;

    // Constructor to initialize the telemetry
    public Dataflow(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void sendDatasToDashBoard(String[] contexts, Object... values) {
        contexts = (contexts == null || contexts.length == 0) ? new String[]{" "} : contexts;
        for (int i = 0; i < values.length; i++) {
            String context = (i < contexts.length && contexts[i] != null && !contexts[i].isEmpty()) ? contexts[i] : "Unknown param";
            dashboardTelemetry.addData(context, values[i]);
        }
        dashboardTelemetry.update();
    }

    public void sendDatasToTelemetry(String[] contexts, Object... values) {
        contexts = (contexts == null || contexts.length == 0) ? new String[]{" "} : contexts;
        for (int i = 0; i < values.length; i++) {
            String context = (i < contexts.length && contexts[i] != null && !contexts[i].isEmpty()) ? contexts[i] : "Unknown param";
            telemetry.addData(context, values[i]);
        }
        telemetry.update();
    }

    public void sendToAll(String[] contexts, Object... values) {
        sendDatasToTelemetry(contexts, values);
        sendDatasToDashBoard(contexts, values);
    }
}
