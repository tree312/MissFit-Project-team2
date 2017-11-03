package com.bridou_n.beaconscanner.utils;

import android.bluetooth.BluetoothAdapter;
import android.content.Context;
import dagger.internal.Factory;
import javax.annotation.Generated;
import javax.inject.Provider;

@Generated(
  value = "dagger.internal.codegen.ComponentProcessor",
  comments = "https://google.github.io/dagger"
)
public final class BluetoothManager_Factory implements Factory<BluetoothManager> {
  private final Provider<BluetoothAdapter> adapterProvider;

  private final Provider<Context> contextProvider;

  public BluetoothManager_Factory(
      Provider<BluetoothAdapter> adapterProvider, Provider<Context> contextProvider) {
    assert adapterProvider != null;
    this.adapterProvider = adapterProvider;
    assert contextProvider != null;
    this.contextProvider = contextProvider;
  }

  @Override
  public BluetoothManager get() {
    return new BluetoothManager(adapterProvider.get(), contextProvider.get());
  }

  public static Factory<BluetoothManager> create(
      Provider<BluetoothAdapter> adapterProvider, Provider<Context> contextProvider) {
    return new BluetoothManager_Factory(adapterProvider, contextProvider);
  }
}
