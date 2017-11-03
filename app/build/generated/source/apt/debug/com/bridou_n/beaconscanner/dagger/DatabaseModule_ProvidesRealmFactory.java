package com.bridou_n.beaconscanner.dagger;

import dagger.internal.Factory;
import dagger.internal.Preconditions;
import io.realm.Realm;
import javax.annotation.Generated;

@Generated(
  value = "dagger.internal.codegen.ComponentProcessor",
  comments = "https://google.github.io/dagger"
)
public final class DatabaseModule_ProvidesRealmFactory implements Factory<Realm> {
  private final DatabaseModule module;

  public DatabaseModule_ProvidesRealmFactory(DatabaseModule module) {
    assert module != null;
    this.module = module;
  }

  @Override
  public Realm get() {
    return Preconditions.checkNotNull(
        module.providesRealm(), "Cannot return null from a non-@Nullable @Provides method");
  }

  public static Factory<Realm> create(DatabaseModule module) {
    return new DatabaseModule_ProvidesRealmFactory(module);
  }
}
