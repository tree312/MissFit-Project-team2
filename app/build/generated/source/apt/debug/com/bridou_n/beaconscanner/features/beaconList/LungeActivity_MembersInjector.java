package com.bridou_n.beaconscanner.features.beaconList;

import android.view.animation.Animation;
import com.bridou_n.beaconscanner.events.RxBus;
import com.bridou_n.beaconscanner.utils.BluetoothManager;
import dagger.MembersInjector;
import io.realm.Realm;
import javax.annotation.Generated;
import javax.inject.Provider;
import org.altbeacon.beacon.BeaconManager;

@Generated(
  value = "dagger.internal.codegen.ComponentProcessor",
  comments = "https://google.github.io/dagger"
)
public final class LungeActivity_MembersInjector implements MembersInjector<LungeActivity> {
  private final Provider<Animation> rotateProvider;

  private final Provider<BluetoothManager> bluetoothProvider;

  private final Provider<BeaconManager> beaconManagerProvider;

  private final Provider<RxBus> rxBusProvider;

  private final Provider<Realm> realmProvider;

  public LungeActivity_MembersInjector(
      Provider<Animation> rotateProvider,
      Provider<BluetoothManager> bluetoothProvider,
      Provider<BeaconManager> beaconManagerProvider,
      Provider<RxBus> rxBusProvider,
      Provider<Realm> realmProvider) {
    assert rotateProvider != null;
    this.rotateProvider = rotateProvider;
    assert bluetoothProvider != null;
    this.bluetoothProvider = bluetoothProvider;
    assert beaconManagerProvider != null;
    this.beaconManagerProvider = beaconManagerProvider;
    assert rxBusProvider != null;
    this.rxBusProvider = rxBusProvider;
    assert realmProvider != null;
    this.realmProvider = realmProvider;
  }

  public static MembersInjector<LungeActivity> create(
      Provider<Animation> rotateProvider,
      Provider<BluetoothManager> bluetoothProvider,
      Provider<BeaconManager> beaconManagerProvider,
      Provider<RxBus> rxBusProvider,
      Provider<Realm> realmProvider) {
    return new LungeActivity_MembersInjector(
        rotateProvider, bluetoothProvider, beaconManagerProvider, rxBusProvider, realmProvider);
  }

  @Override
  public void injectMembers(LungeActivity instance) {
    if (instance == null) {
      throw new NullPointerException("Cannot inject members into a null reference");
    }
    instance.rotate = rotateProvider.get();
    instance.bluetooth = bluetoothProvider.get();
    instance.beaconManager = beaconManagerProvider.get();
    instance.rxBus = rxBusProvider.get();
    instance.realm = realmProvider.get();
  }

  public static void injectRotate(LungeActivity instance, Provider<Animation> rotateProvider) {
    instance.rotate = rotateProvider.get();
  }

  public static void injectBluetooth(
      LungeActivity instance, Provider<BluetoothManager> bluetoothProvider) {
    instance.bluetooth = bluetoothProvider.get();
  }

  public static void injectBeaconManager(
      LungeActivity instance, Provider<BeaconManager> beaconManagerProvider) {
    instance.beaconManager = beaconManagerProvider.get();
  }

  public static void injectRxBus(LungeActivity instance, Provider<RxBus> rxBusProvider) {
    instance.rxBus = rxBusProvider.get();
  }

  public static void injectRealm(LungeActivity instance, Provider<Realm> realmProvider) {
    instance.realm = realmProvider.get();
  }
}
