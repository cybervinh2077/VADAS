package com.vadas.feature.auto;

import android.content.Intent;

import androidx.annotation.NonNull;
import androidx.car.app.Screen;
import androidx.car.app.Session;

/**
 * Android Auto session lifecycle manager for VADAS.
 */
public class VADASCarSession extends Session {

    @NonNull
    @Override
    public Screen onCreateScreen(@NonNull Intent intent) {
        return new VADASDashboardScreen(getCarContext());
    }
}
