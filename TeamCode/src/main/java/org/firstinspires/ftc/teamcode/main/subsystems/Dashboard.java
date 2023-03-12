package org.firstinspires.ftc.teamcode.main.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Static class for the FTC Dashboard.
 *
 * This class allows us to access packets and the canvas, allowing multiple classes to send telemetry
 */
public class Dashboard {
   public static TelemetryPacket packet = new TelemetryPacket();
   public static Canvas fieldOverlay = packet.fieldOverlay();
   private static FtcDashboard dashboard = null;

   public static void setUp() {
      dashboard = FtcDashboard.getInstance();
   }

   public static void resetPacket() {
      packet = new TelemetryPacket();
      fieldOverlay = packet.fieldOverlay();
   }

   public static void sendPacket() {
      if (dashboard == null) {
         setUp();
      }

      if (dashboard != null) {
         dashboard.sendTelemetryPacket(packet);
         resetPacket();
      } else {
         Log.e("Dashboard", "Dashboard failed to init - Dashboard is null");
      }
   }

   public static FtcDashboard getInstance() {
      return dashboard;
   }
}