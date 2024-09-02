package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Dataflow {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    Telemetry telemetry;

    public Dataflow(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void addDatasToDashBoard(String[] contexts, Object... values) {
        contexts = (contexts == null || contexts.length == 0) ? new String[]{" "} : contexts;
        for (int i = 0; i < values.length; i++) {
            String context = (i < contexts.length && contexts[i] != null && !contexts[i].isEmpty()) ? contexts[i] : "Unknown param";
            dashboardTelemetry.addData(context, values[i]);
        }
    }

    public void addDatasToTelemetry(String[] contexts, Object... values) {
        contexts = (contexts == null || contexts.length == 0) ? new String[]{" "} : contexts;
        for (int i = 0; i < values.length; i++) {
            String context = (i < contexts.length && contexts[i] != null && !contexts[i].isEmpty()) ? contexts[i] : "Unknown param";
            telemetry.addData(context, values[i]);
        }
    }

    public void addToAll(String[] contexts, Object... values) {
        addDatasToTelemetry(contexts, values);
        addDatasToDashBoard(contexts, values);
    }
    public void sendDatas() {
        telemetry.update();
        dashboardTelemetry.update();
    }
}
