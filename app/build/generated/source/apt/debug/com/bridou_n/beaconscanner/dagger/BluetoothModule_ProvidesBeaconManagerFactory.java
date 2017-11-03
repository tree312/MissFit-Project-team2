package com.bridou_n.beaconscanner.dagger;

import android.content.Context;
import dagger.internal.Factory;
import dagger.internal.Preconditions;
import javax.annotation.Generated;
import javax.inject.Provider;
import org.altbeacon.beacon.BeaconManager;

@Generated(
  value = "dagger.internal.codegen.ComponentProcessor",
  comments = "https://google.github.io/dagger"
)
public final class BluetoothModule_ProvidesBeaconManagerFactory implements Factory<BeaconManager> {
  private final BluetoothModule module;

  private final Provider<Context> ctxProvider;

  public BluetoothModule_ProvidesBeaconManagerFactory(
      BluetoothModule module, Provider<Context> ctxProvider) {
    assert module != null;
    this.module = module;
    assert ctxProvider != null;
    this.ctxProvider = ctxProvider;
  }

  @Override
  public BeaconManager get() {
    return Preconditions.checkNotNull(
        module.providesBeaconManager(ctxProvider.get()),
        "Cannot return null from a non-@Nullable @Provides method");
  }

  public static Factory<BeaconManager> create(
      BluetoothModule module, Provider<Context> ctxProvider) {
    return new BluetoothModule_ProvidesBeaconManagerFactory(module, ctxProvider);
  }
}
