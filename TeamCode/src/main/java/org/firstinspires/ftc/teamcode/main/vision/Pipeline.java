package org.firstinspires.ftc.teamcode.main.vision;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * Base class for similar detection pipelines
 *
 * Includes saving and retrieving images through dashboard, and some things for image pre-processing
 * However usage of image files and image correctors must be implemented in the subclass
 */
public abstract class Pipeline extends OpenCvPipeline {
    protected boolean isInitialized = false;

    private static final String savePath = "/sdcard/EasyOpenCV";
    public static boolean SAVE_IMAGE = false;
    public static int READ_IMAGE = -1; // indexed from 1
    public static boolean INDEX_IMAGES = false;

    public static boolean WHITE_BALANCE_IMAGE = true;
    public static boolean GAMMA_CORRECT_IMAGE = true;

    private List<String> indexedFiles = new ArrayList<>(); // contains file names

    public Pipeline() {
    }

    @Override
    public void init(Mat mat) {
        isInitialized = true;
        indexImageFiles();
        READ_IMAGE = -indexedFiles.size();
    }

    public abstract void setGammaValue(double gammaValue);
    public abstract double getGammaValue();

    public abstract int getReading();

    public boolean isInitialized() {
        return isInitialized;
    }

    public Mat getImageFile(int imageNumber) {
        Log.d("TSEPipeline", "getting mat (" + imageNumber + " of " + indexedFiles.size() + ")");
        if (imageNumber < 1 || imageNumber > indexedFiles.size()) return null;

        try {
            String filename = String.format("%s/%s", savePath, indexedFiles.get(imageNumber - 1));
            Log.d("TSEPipeline", "trying to retrieve: " + filename);
            Mat output = Imgcodecs.imread(filename);
            Imgproc.cvtColor(output, output, Imgproc.COLOR_RGB2BGR);
            return output;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public void saveMat(Mat mat, String name) {
        String time = new SimpleDateFormat("yyyy-MM-dd HH-mm-ss-SSS", Locale.getDefault()).format(new Date());
        String filename = String.format("%s %s (%s)", time, name, this.getClass().getSimpleName());

        if (!new File(String.format("%s/%s.png", savePath, filename)).exists()) {
            saveMatToDiskFullPath(mat, String.format("%s/%s.png", savePath, filename));
            Log.d("TSEPipeline", "saved mat '" + String.format("%s/%s.png", savePath, filename) + "'");
        } else {
            int addon = 2;
            while (new File(String.format("%s/%s %d.png", savePath, filename, addon)).exists()) {
                addon += 1;
            }
            saveMatToDiskFullPath(mat, String.format("%s/%s %d.png", savePath, filename, addon));
            Log.d("TSEPipeline", "saved mat '" + String.format("%s/%s %d.png", savePath, filename, addon) + "'");
        }
        SAVE_IMAGE = false;
    }

    public void indexImageFiles() {
        try {
            indexedFiles = Stream.of(new File(savePath).list((dir, name) -> name.endsWith(".png"))).sorted(Comparator.reverseOrder()).collect(Collectors.toList());
        } catch (Exception e) {
            e.printStackTrace();
            indexedFiles.clear();
        }
        INDEX_IMAGES = false;
    }

    protected Mat notNull(Mat checkedMat, Mat defaultMat) {
        if (checkedMat == null || checkedMat.empty()) return defaultMat;
        return checkedMat;
    }
}
