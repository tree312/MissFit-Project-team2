package com.bridou_n.beaconscanner.dagger;

import android.content.Context;
import dagger.internal.Factory;
import dagger.internal.Preconditions;
import javax.annotation.Generated;

@Generated(
  value = "dagger.internal.codegen.ComponentProcessor",
  comments = "https://google.github.io/dagger"
)
public final class ContextModule_ProvidesContextFactory implements Factory<Context> {
  private final ContextModule module;

  public ContextModule_ProvidesContextFactory(ContextModule module) {
    assert module != null;
    this.module = module;
  }

  @Override
  public Context get() {
    return Preconditions.checkNotNull(
        module.providesContext(), "Cannot return null from a non-@Nullable @Provides method");
  }

  public static Factory<Context> create(ContextModule module) {
    return new ContextModule_ProvidesContextFactory(module);
  }
}
