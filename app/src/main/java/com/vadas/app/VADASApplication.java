package com.vadas.app;

import android.app.Application;
import android.util.Log;

public class VADASApplication extends Application {

    private static final String TAG = "VADASApp";

    @Override
    public void onCreate() {
        super.onCreate();
        Log.i(TAG, "Starting VADAS Application - Vietnam Advanced Driver Assistance System");
    }
}
