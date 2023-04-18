package org.firstinspires.ftc.teamcode.main.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.util.data.FakeDashboardPacket;

/**
 * Static class for the FTC Dashboard.
 *
 * This class allows us to access packets and the canvas, allowing multiple classes to send telemetry
 */
public class Dashboard {
   public static TelemetryPacket packet = new TelemetryPacket();
   public static Canvas fieldOverlay = packet.fieldOverlay();
   private static FtcDashboard dashboard = null;

   public static boolean DISABLE_DASHBOARD = true;
   public static boolean DISABLE_CANVAS = true;

   public static void setUp() {
      dashboard = FtcDashboard.getInstance();
      resetPacket();
   }

   public static void resetPacket() {
      if (DISABLE_DASHBOARD) {
         packet = new FakeDashboardPacket();
         fieldOverlay = packet.fieldOverlay();
      } else {
         packet = new TelemetryPacket();
         fieldOverlay = packet.fieldOverlay();
      }
   }

   public static void sendPacket() {
      if (DISABLE_DASHBOARD) return;
      if (dashboard == null) throw new RuntimeException("Dashboard sendPacket() called before setUp()");

      dashboard.sendTelemetryPacket(packet);
      resetPacket();
   }

   public static FtcDashboard getInstance() { // not protected from disable
      return dashboard;
   }

   public static void startCameraStream(CameraStreamSource source, double maxFps) {
      if (DISABLE_DASHBOARD) return;
      if (dashboard == null) throw new RuntimeException("Dashboard startCameraStream() called before setUp()");
      dashboard.startCameraStream(source, maxFps);
   }

   public static void stopCameraStream() {
      if (DISABLE_DASHBOARD) return;
      if (dashboard == null) throw new RuntimeException("Dashboard stopCameraStream() called before setUp()");
      dashboard.stopCameraStream();
   }

   public static void setTelemetryTransmissionInterval(int newTransmissionInterval) {
      if (DISABLE_DASHBOARD) return;
      if (dashboard == null) throw new RuntimeException("Dashboard setTelemetryTransmissionInterval() called before setUp()");
      dashboard.setTelemetryTransmissionInterval(newTransmissionInterval);
   }
}