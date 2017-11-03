package com.bridou_n.beaconscanner.dagger;

import android.bluetooth.BluetoothAdapter;
import android.support.annotation.Nullable;
import dagger.internal.Factory;
import javax.annotation.Generated;

@Generated(
  value = "dagger.internal.codegen.ComponentProcessor",
  comments = "https://google.github.io/dagger"
)
public final class BluetoothModule_ProvidesBluetoothAdapterFactory
    implements Factory<BluetoothAdapter> {
  private final BluetoothModule module;

  public BluetoothModule_ProvidesBluetoothAdapterFactory(BluetoothModule module) {
    assert module != null;
    this.module = module;
  }

  @Override
  @Nullable
  public BluetoothAdapter get() {
    return module.providesBluetoothAdapter();
  }

  public static Factory<BluetoothAdapter> create(BluetoothModule module) {
    return new BluetoothModule_ProvidesBluetoothAdapterFactory(module);
  }
}
