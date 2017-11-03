package com.bridou_n.beaconscanner.dagger;

import com.bridou_n.beaconscanner.features.beaconList.LungeActivity;
import com.bridou_n.beaconscanner.features.beaconList.MainActivity;
import com.bridou_n.beaconscanner.features.beaconList.SquartActivity;

import dagger.Component;

/**
 * Created by bridou_n on 08/10/2016.
 */
@PerActivity
@Component(dependencies = AppComponent.class,
        modules = {
                BluetoothModule.class,
                AnimationModule.class
        })
public interface ActivityComponent {

    void inject(MainActivity activity);

    void inject(SquartActivity activity);

    void inject(LungeActivity activity);
}