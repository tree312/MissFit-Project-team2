package com.bridou_n.beaconscanner.dagger;

import android.bluetooth.BluetoothAdapter;
import android.content.Context;
import android.view.animation.Animation;
import com.bridou_n.beaconscanner.events.RxBus;
import com.bridou_n.beaconscanner.features.beaconList.LungeActivity;
import com.bridou_n.beaconscanner.features.beaconList.LungeActivity_MembersInjector;
import com.bridou_n.beaconscanner.features.beaconList.MainActivity;
import com.bridou_n.beaconscanner.features.beaconList.MainActivity_MembersInjector;
import com.bridou_n.beaconscanner.features.beaconList.SquartActivity;
import com.bridou_n.beaconscanner.features.beaconList.SquartActivity_MembersInjector;
import com.bridou_n.beaconscanner.utils.BluetoothManager;
import com.bridou_n.beaconscanner.utils.BluetoothManager_Factory;
import dagger.MembersInjector;
import dagger.internal.DoubleCheck;
import dagger.internal.Factory;
import dagger.internal.Preconditions;
import io.realm.Realm;
import javax.annotation.Generated;
import javax.inject.Provider;
import org.altbeacon.beacon.BeaconManager;

@Generated(
  value = "dagger.internal.codegen.ComponentProcessor",
  comments = "https://google.github.io/dagger"
)
public final class DaggerActivityComponent implements ActivityComponent {
  private Provider<Context> contextProvider;

  private Provider<Animation> providesFabSearchAnimationProvider;

  private Provider<BluetoothAdapter> providesBluetoothAdapterProvider;

  private Provider<BluetoothManager> bluetoothManagerProvider;

  private Provider<BeaconManager> providesBeaconManagerProvider;

  private Provider<RxBus> rxBusProvider;

  private Provider<Realm> realmProvider;

  private MembersInjector<MainActivity> mainActivityMembersInjector;

  private MembersInjector<SquartActivity> squartActivityMembersInjector;

  private MembersInjector<LungeActivity> lungeActivityMembersInjector;

  private DaggerActivityComponent(Builder builder) {
    assert builder != null;
    initialize(builder);
  }

  public static Builder builder() {
    return new Builder();
  }

  @SuppressWarnings("unchecked")
  private void initialize(final Builder builder) {

    this.contextProvider =
        new Factory<Context>() {
          private final AppComponent appComponent = builder.appComponent;

          @Override
          public Context get() {
            return Preconditions.checkNotNull(
                appComponent.context(), "Cannot return null from a non-@Nullable component method");
          }
        };

    this.providesFabSearchAnimationProvider =
        DoubleCheck.provider(
            AnimationModule_ProvidesFabSearchAnimationFactory.create(
                builder.animationModule, contextProvider));

    this.providesBluetoothAdapterProvider =
        DoubleCheck.provider(
            BluetoothModule_ProvidesBluetoothAdapterFactory.create(builder.bluetoothModule));

    this.bluetoothManagerProvider =
        BluetoothManager_Factory.create(providesBluetoothAdapterProvider, contextProvider);

    this.providesBeaconManagerProvider =
        DoubleCheck.provider(
            BluetoothModule_ProvidesBeaconManagerFactory.create(
                builder.bluetoothModule, contextProvider));

    this.rxBusProvider =
        new Factory<RxBus>() {
          private final AppComponent appComponent = builder.appComponent;

          @Override
          public RxBus get() {
            return Preconditions.checkNotNull(
                appComponent.rxBus(), "Cannot return null from a non-@Nullable component method");
          }
        };

    this.realmProvider =
        new Factory<Realm>() {
          private final AppComponent appComponent = builder.appComponent;

          @Override
          public Realm get() {
            return Preconditions.checkNotNull(
                appComponent.realm(), "Cannot return null from a non-@Nullable component method");
          }
        };

    this.mainActivityMembersInjector =
        MainActivity_MembersInjector.create(
            providesFabSearchAnimationProvider,
            bluetoothManagerProvider,
            providesBeaconManagerProvider,
            rxBusProvider,
            realmProvider);

    this.squartActivityMembersInjector =
        SquartActivity_MembersInjector.create(
            providesFabSearchAnimationProvider,
            bluetoothManagerProvider,
            providesBeaconManagerProvider,
            rxBusProvider,
            realmProvider);

    this.lungeActivityMembersInjector =
        LungeActivity_MembersInjector.create(
            providesFabSearchAnimationProvider,
            bluetoothManagerProvider,
            providesBeaconManagerProvider,
            rxBusProvider,
            realmProvider);
  }

  @Override
  public void inject(MainActivity activity) {
    mainActivityMembersInjector.injectMembers(activity);
  }

  @Override
  public void inject(SquartActivity activity) {
    squartActivityMembersInjector.injectMembers(activity);
  }

  @Override
  public void inject(LungeActivity activity) {
    lungeActivityMembersInjector.injectMembers(activity);
  }

  public static final class Builder {
    private AnimationModule animationModule;

    private BluetoothModule bluetoothModule;

    private AppComponent appComponent;

    private Builder() {}

    public ActivityComponent build() {
      if (animationModule == null) {
        this.animationModule = new AnimationModule();
      }
      if (bluetoothModule == null) {
        this.bluetoothModule = new BluetoothModule();
      }
      if (appComponent == null) {
        throw new IllegalStateException(AppComponent.class.getCanonicalName() + " must be set");
      }
      return new DaggerActivityComponent(this);
    }

    public Builder bluetoothModule(BluetoothModule bluetoothModule) {
      this.bluetoothModule = Preconditions.checkNotNull(bluetoothModule);
      return this;
    }

    public Builder animationModule(AnimationModule animationModule) {
      this.animationModule = Preconditions.checkNotNull(animationModule);
      return this;
    }

    public Builder appComponent(AppComponent appComponent) {
      this.appComponent = Preconditions.checkNotNull(appComponent);
      return this;
    }
  }
}
