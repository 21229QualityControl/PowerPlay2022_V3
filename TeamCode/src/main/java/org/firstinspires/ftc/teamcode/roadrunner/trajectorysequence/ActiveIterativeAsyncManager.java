package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence;

import java.util.ArrayList;
import java.util.ConcurrentModificationException;

/**
 * Manages all the active iterative functions for the follower
 */
public class ActiveIterativeAsyncManager {
    private static ArrayList<IterativeAsync> activeIterativeAsyncs = new ArrayList<>();

    public static void runAll() {
        try {
            for (IterativeAsync async : activeIterativeAsyncs) {
                async.run();
            }
        } catch (ConcurrentModificationException ignored) {
        }
    }

    public static void clear() {
        activeIterativeAsyncs.clear();
    }

    public static boolean add(String name, Runnable iterativeAsyncFunction) {
        if (iterativeAsyncFunction == null) return false;
        activeIterativeAsyncs.add(new IterativeAsync(name, iterativeAsyncFunction));
        return true;
    }

    public static boolean remove(String name) {
        for (int i = 0; i < activeIterativeAsyncs.size(); i++) {
            if (activeIterativeAsyncs.get(i).getName().equals(name)) {
                activeIterativeAsyncs.remove(i);
                return true;
            }
        }
        return false;
    }

    private static class IterativeAsync {
        String name;
        Runnable iterativeAsyncFunction;

        IterativeAsync(String name, Runnable iterativeAsyncFunction) {
            this.name = name;
            this.iterativeAsyncFunction = iterativeAsyncFunction;
        }

        private void run() {
            iterativeAsyncFunction.run();
        }

        private String getName() {
            return name;
        }
    }
}
