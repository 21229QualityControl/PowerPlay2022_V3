package org.firstinspires.ftc.teamcode.util.data;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class FakeDashboardPacket extends TelemetryPacket {
    private Canvas fieldOverlay;

    /**
     * Creates a new telemetry packet.
     */
    public FakeDashboardPacket() {
        fieldOverlay = new FakeDashboardCanvas();
    }

    /**
     * Stores a single key-value pair.
     * @param key
     * @param value
     */
    public void put(String key, Object value) {
    }

    /**
     * Stores all entries of the provided map.
     * @param map
     */
    public void putAll(Map<String, Object> map) {
    }

    /**
     * Adds a line to the telemetry log.
     * @param line
     */
    public void addLine(String line) {
    }

    /**
     * Clears the telemetry log.
     */
    public void clearLines() {
    }

    /**
     * Adds and returns the current timestamp to the packet. This is called automatically when the
     * packet is sent (and any previous timestamp will be overwritten).
     */
    public long addTimestamp() {
        return 0;
    }

    /**
     * Returns the field overlay canvas.
     */
    public Canvas fieldOverlay() {
        return fieldOverlay;
    }

    /**
     * Adapter to use dashboard telemetry like normal SDK telemetry. Note that this doesn't support
     * all of the operations yet.
     */
    public static class Adapter implements Telemetry {

        public Adapter(FtcDashboard dashboard) {
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            return null;
        }

        @Override
        public Item addData(String caption, Object value) {
            return null;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            throw new UnsupportedOperationException();
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            throw new UnsupportedOperationException();
        }

        @Override
        public boolean removeItem(Item item) {
            throw new UnsupportedOperationException();
        }

        @Override
        public void clear() {
        }

        @Override
        public void clearAll() {}

        @Override
        public Object addAction(Runnable action) {
            throw new UnsupportedOperationException();
        }

        @Override
        public boolean removeAction(Object token) {
            throw new UnsupportedOperationException();
        }

        @Override
        public void speak(String text) {
            throw new UnsupportedOperationException();
        }

        @Override
        public void speak(String text, String languageCode, String countryCode) {
            throw new UnsupportedOperationException();
        }

        @Override
        public boolean update() {
            return true;
        }

        @Override
        public Line addLine() {
            return null;
        }

        @Override
        public Line addLine(String lineCaption) {
            return null;
        }

        @Override
        public boolean removeLine(Line line) {
            throw new UnsupportedOperationException();
        }

        @Override
        public boolean isAutoClear() {
            return false;
        }

        @Override
        public void setAutoClear(boolean autoClear) {
            throw new UnsupportedOperationException();
        }

        @Override
        public int getMsTransmissionInterval() {
            return 0;
        }

        @Override
        public void setMsTransmissionInterval(int msTransmissionInterval) {
        }

        @Override
        public String getItemSeparator() {
            return null;
        }

        @Override
        public void setItemSeparator(String itemSeparator) {

        }

        @Override
        public String getCaptionValueSeparator() {
            return null;
        }

        @Override
        public void setCaptionValueSeparator(String captionValueSeparator) {

        }

        @Override
        public void setDisplayFormat(DisplayFormat displayFormat) {

        }

        @Override
        public Log log() {
            return null;
        }
    }
}
