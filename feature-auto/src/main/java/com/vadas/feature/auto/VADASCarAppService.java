package com.vadas.feature.auto;

import androidx.annotation.NonNull;
import androidx.car.app.CarAppService;
import androidx.car.app.Session;
import androidx.car.app.validation.HostValidator;

/**
 * Android Auto CarAppService for VADAS.
 * Conforms to Google Driving Safety & Distraction Guidelines:
 * No raw camera feeds; clean, glanceable driving safety metrics and threat alerts.
 */
public class VADASCarAppService extends CarAppService {

    @NonNull
    @Override
    public HostValidator createHostValidator() {
        // Allows both production Android Auto Head Units and Desktop Head Unit (DHU) emulator
        return HostValidator.ALLOW_ALL_HOSTS_VALIDATOR;
    }

    @NonNull
    @Override
    public Session onCreateSession() {
        return new VADASCarSession();
    }
}
