package com.bridou_n.beaconscanner.dagger;

import android.content.Context;
import android.view.animation.Animation;
import dagger.internal.Factory;
import dagger.internal.Preconditions;
import javax.annotation.Generated;
import javax.inject.Provider;

@Generated(
  value = "dagger.internal.codegen.ComponentProcessor",
  comments = "https://google.github.io/dagger"
)
public final class AnimationModule_ProvidesFabSearchAnimationFactory implements Factory<Animation> {
  private final AnimationModule module;

  private final Provider<Context> ctxProvider;

  public AnimationModule_ProvidesFabSearchAnimationFactory(
      AnimationModule module, Provider<Context> ctxProvider) {
    assert module != null;
    this.module = module;
    assert ctxProvider != null;
    this.ctxProvider = ctxProvider;
  }

  @Override
  public Animation get() {
    return Preconditions.checkNotNull(
        module.providesFabSearchAnimation(ctxProvider.get()),
        "Cannot return null from a non-@Nullable @Provides method");
  }

  public static Factory<Animation> create(AnimationModule module, Provider<Context> ctxProvider) {
    return new AnimationModule_ProvidesFabSearchAnimationFactory(module, ctxProvider);
  }
}
