package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ActiveLinearAsyncManager {
    private static ArrayList<LinearAsync> activeLinearAsyncs = new ArrayList<>();

    public static void terminateAll() {
        for (LinearAsync async : activeLinearAsyncs) async.terminate();
        activeLinearAsyncs.clear();
    }

    public static boolean add(String name, Runnable linearAsyncFunction) {
        if (linearAsyncFunction == null) return false;
        LinearAsync async = new LinearAsync(name, linearAsyncFunction);
        activeLinearAsyncs.add(async);
        async.run();
        return true;
    }

    public static boolean isFinished(String name) {
        Iterator<LinearAsync> iterator = activeLinearAsyncs.iterator();
        while (iterator.hasNext()) {
            LinearAsync linearAsync = iterator.next();
            if (linearAsync.getName().equals(name)) { // find async
                // if async is done, remove from list and return true
                if (linearAsync.executor.isTerminated()) {
                    iterator.remove();
                    return true;
                } else { // async is in list and not done, return false
                    return false;
                }
            }
        }
        // async isn't in the list, return true
        return true;
    }

    private static class LinearAsync {
        String name;
        Runnable linearAsyncFunction;
        ExecutorService executor;

        LinearAsync(String name, Runnable linearAsyncFunction) {
            this.name = name;
            this.linearAsyncFunction = linearAsyncFunction;
        }

        private void run() {
            executor = Executors.newSingleThreadExecutor();
            executor.submit(linearAsyncFunction);
            executor.shutdown();
        }

        private void terminate() {
            executor.shutdownNow();
        }

        private String getName() {
            return name;
        }
    }
}
