// Generated code from Butter Knife. Do not modify!
package com.bridou_n.beaconscanner.features.beaconList;

import android.support.annotation.CallSuper;
import android.support.annotation.UiThread;
import android.support.design.widget.CoordinatorLayout;
import android.support.design.widget.FloatingActionButton;
import android.support.v7.widget.Toolbar;
import android.view.View;
import android.widget.ImageButton;
import android.widget.ProgressBar;
import android.widget.TextView;
import butterknife.Unbinder;
import butterknife.internal.DebouncingOnClickListener;
import butterknife.internal.Utils;
import com.bridou_n.beaconscanner.R;
import java.lang.IllegalStateException;
import java.lang.Override;

public class LungeActivity_ViewBinding<T extends LungeActivity> implements Unbinder {
  protected T target;

  private View view2131558531;

  @UiThread
  public LungeActivity_ViewBinding(final T target, View source) {
    this.target = target;

    View view;
    target.toolbar = Utils.findRequiredViewAsType(source, R.id.toolbar, "field 'toolbar'", Toolbar.class);
    target.rootView = Utils.findRequiredViewAsType(source, R.id.activity_lunge, "field 'rootView'", CoordinatorLayout.class);
    target.bluetoothState = Utils.findRequiredViewAsType(source, R.id.bluetooth_state, "field 'bluetoothState'", TextView.class);
    view = Utils.findRequiredView(source, R.id.scan_fab, "field 'scanFab' and method 'startStopScan'");
    target.scanFab = Utils.castView(view, R.id.scan_fab, "field 'scanFab'", FloatingActionButton.class);
    view2131558531 = view;
    view.setOnClickListener(new DebouncingOnClickListener() {
      @Override
      public void doClick(View p0) {
        target.startStopScan();
      }
    });
    target.scanProgress = Utils.findRequiredViewAsType(source, R.id.scan_progress, "field 'scanProgress'", ProgressBar.class);
    target.sButton = Utils.findRequiredViewAsType(source, R.id.exStartButton, "field 'sButton'", ImageButton.class);
    target.rButton = Utils.findRequiredViewAsType(source, R.id.exResetButton, "field 'rButton'", ImageButton.class);
    target.rTextView = Utils.findRequiredViewAsType(source, R.id.resultTxt, "field 'rTextView'", TextView.class);
    target.tTextView = Utils.findRequiredViewAsType(source, R.id.timeTxt, "field 'tTextView'", TextView.class);
    target.nTextView = Utils.findRequiredViewAsType(source, R.id.countTxt, "field 'nTextView'", TextView.class);
    target.legTextView = Utils.findRequiredViewAsType(source, R.id.textview_leg, "field 'legTextView'", TextView.class);
    target.progressBar = Utils.findRequiredViewAsType(source, R.id.progressBar, "field 'progressBar'", ProgressBar.class);
  }

  @Override
  @CallSuper
  public void unbind() {
    T target = this.target;
    if (target == null) throw new IllegalStateException("Bindings already cleared.");

    target.toolbar = null;
    target.rootView = null;
    target.bluetoothState = null;
    target.scanFab = null;
    target.scanProgress = null;
    target.sButton = null;
    target.rButton = null;
    target.rTextView = null;
    target.tTextView = null;
    target.nTextView = null;
    target.legTextView = null;
    target.progressBar = null;

    view2131558531.setOnClickListener(null);
    view2131558531 = null;

    this.target = null;
  }
}
