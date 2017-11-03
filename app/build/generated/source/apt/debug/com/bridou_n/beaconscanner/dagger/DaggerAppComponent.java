package com.bridou_n.beaconscanner.dagger;

import android.content.Context;
import com.bridou_n.beaconscanner.events.RxBus;
import dagger.internal.DoubleCheck;
import dagger.internal.Preconditions;
import io.realm.Realm;
import javax.annotation.Generated;
import javax.inject.Provider;

@Generated(
  value = "dagger.internal.codegen.ComponentProcessor",
  comments = "https://google.github.io/dagger"
)
public final class DaggerAppComponent implements AppComponent {
  private Provider<Context> providesContextProvider;

  private Provider<Realm> providesRealmProvider;

  private Provider<RxBus> providesRxBusProvider;

  private DaggerAppComponent(Builder builder) {
    assert builder != null;
    initialize(builder);
  }

  public static Builder builder() {
    return new Builder();
  }

  @SuppressWarnings("unchecked")
  private void initialize(final Builder builder) {

    this.providesContextProvider =
        DoubleCheck.provider(ContextModule_ProvidesContextFactory.create(builder.contextModule));

    this.providesRealmProvider = DatabaseModule_ProvidesRealmFactory.create(builder.databaseModule);

    this.providesRxBusProvider =
        DoubleCheck.provider(EventModule_ProvidesRxBusFactory.create(builder.eventModule));
  }

  @Override
  public Context context() {
    return providesContextProvider.get();
  }

  @Override
  public Realm realm() {
    return providesRealmProvider.get();
  }

  @Override
  public RxBus rxBus() {
    return providesRxBusProvider.get();
  }

  public static final class Builder {
    private ContextModule contextModule;

    private DatabaseModule databaseModule;

    private EventModule eventModule;

    private Builder() {}

    public AppComponent build() {
      if (contextModule == null) {
        throw new IllegalStateException(ContextModule.class.getCanonicalName() + " must be set");
      }
      if (databaseModule == null) {
        this.databaseModule = new DatabaseModule();
      }
      if (eventModule == null) {
        this.eventModule = new EventModule();
      }
      return new DaggerAppComponent(this);
    }

    public Builder contextModule(ContextModule contextModule) {
      this.contextModule = Preconditions.checkNotNull(contextModule);
      return this;
    }

    public Builder databaseModule(DatabaseModule databaseModule) {
      this.databaseModule = Preconditions.checkNotNull(databaseModule);
      return this;
    }

    public Builder eventModule(EventModule eventModule) {
      this.eventModule = Preconditions.checkNotNull(eventModule);
      return this;
    }
  }
}
