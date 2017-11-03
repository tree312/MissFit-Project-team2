package com.bridou_n.beaconscanner.dagger;

import com.bridou_n.beaconscanner.events.RxBus;
import dagger.internal.Factory;
import dagger.internal.Preconditions;
import javax.annotation.Generated;

@Generated(
  value = "dagger.internal.codegen.ComponentProcessor",
  comments = "https://google.github.io/dagger"
)
public final class EventModule_ProvidesRxBusFactory implements Factory<RxBus> {
  private final EventModule module;

  public EventModule_ProvidesRxBusFactory(EventModule module) {
    assert module != null;
    this.module = module;
  }

  @Override
  public RxBus get() {
    return Preconditions.checkNotNull(
        module.providesRxBus(), "Cannot return null from a non-@Nullable @Provides method");
  }

  public static Factory<RxBus> create(EventModule module) {
    return new EventModule_ProvidesRxBusFactory(module);
  }
}
