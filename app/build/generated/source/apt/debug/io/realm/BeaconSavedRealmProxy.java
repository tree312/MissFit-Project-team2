package io.realm;


import android.annotation.TargetApi;
import android.os.Build;
import android.util.JsonReader;
import android.util.JsonToken;
import io.realm.RealmObjectSchema;
import io.realm.RealmSchema;
import io.realm.exceptions.RealmMigrationNeededException;
import io.realm.internal.ColumnInfo;
import io.realm.internal.LinkView;
import io.realm.internal.RealmObjectProxy;
import io.realm.internal.Row;
import io.realm.internal.SharedRealm;
import io.realm.internal.Table;
import io.realm.internal.TableOrView;
import io.realm.internal.android.JsonUtils;
import io.realm.log.RealmLog;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class BeaconSavedRealmProxy extends com.bridou_n.beaconscanner.models.BeaconSaved
    implements RealmObjectProxy, BeaconSavedRealmProxyInterface {

    static final class BeaconSavedColumnInfo extends ColumnInfo
        implements Cloneable {

        public long hashcodeIndex;
        public long beaconTypeIndex;
        public long beaconAddressIndex;
        public long UUIDIndex;
        public long MajorIndex;
        public long MinorIndex;
        public long txPowerIndex;
        public long RSSIIndex;
        public long distanceIndex;
        public long lastSeenIndex;
        public long lastMinuteSeenIndex;
        public long manufacturerIndex;
        public long URLIndex;
        public long namespaceIdIndex;
        public long instanceIdIndex;
        public long hasTelemetryDataIndex;
        public long telemetryVersionIndex;
        public long batteryMilliVoltsIndex;
        public long temperatureIndex;
        public long pduCountIndex;
        public long uptimeIndex;

        BeaconSavedColumnInfo(String path, Table table) {
            final Map<String, Long> indicesMap = new HashMap<String, Long>(21);
            this.hashcodeIndex = getValidColumnIndex(path, table, "BeaconSaved", "hashcode");
            indicesMap.put("hashcode", this.hashcodeIndex);
            this.beaconTypeIndex = getValidColumnIndex(path, table, "BeaconSaved", "beaconType");
            indicesMap.put("beaconType", this.beaconTypeIndex);
            this.beaconAddressIndex = getValidColumnIndex(path, table, "BeaconSaved", "beaconAddress");
            indicesMap.put("beaconAddress", this.beaconAddressIndex);
            this.UUIDIndex = getValidColumnIndex(path, table, "BeaconSaved", "UUID");
            indicesMap.put("UUID", this.UUIDIndex);
            this.MajorIndex = getValidColumnIndex(path, table, "BeaconSaved", "Major");
            indicesMap.put("Major", this.MajorIndex);
            this.MinorIndex = getValidColumnIndex(path, table, "BeaconSaved", "Minor");
            indicesMap.put("Minor", this.MinorIndex);
            this.txPowerIndex = getValidColumnIndex(path, table, "BeaconSaved", "txPower");
            indicesMap.put("txPower", this.txPowerIndex);
            this.RSSIIndex = getValidColumnIndex(path, table, "BeaconSaved", "RSSI");
            indicesMap.put("RSSI", this.RSSIIndex);
            this.distanceIndex = getValidColumnIndex(path, table, "BeaconSaved", "distance");
            indicesMap.put("distance", this.distanceIndex);
            this.lastSeenIndex = getValidColumnIndex(path, table, "BeaconSaved", "lastSeen");
            indicesMap.put("lastSeen", this.lastSeenIndex);
            this.lastMinuteSeenIndex = getValidColumnIndex(path, table, "BeaconSaved", "lastMinuteSeen");
            indicesMap.put("lastMinuteSeen", this.lastMinuteSeenIndex);
            this.manufacturerIndex = getValidColumnIndex(path, table, "BeaconSaved", "manufacturer");
            indicesMap.put("manufacturer", this.manufacturerIndex);
            this.URLIndex = getValidColumnIndex(path, table, "BeaconSaved", "URL");
            indicesMap.put("URL", this.URLIndex);
            this.namespaceIdIndex = getValidColumnIndex(path, table, "BeaconSaved", "namespaceId");
            indicesMap.put("namespaceId", this.namespaceIdIndex);
            this.instanceIdIndex = getValidColumnIndex(path, table, "BeaconSaved", "instanceId");
            indicesMap.put("instanceId", this.instanceIdIndex);
            this.hasTelemetryDataIndex = getValidColumnIndex(path, table, "BeaconSaved", "hasTelemetryData");
            indicesMap.put("hasTelemetryData", this.hasTelemetryDataIndex);
            this.telemetryVersionIndex = getValidColumnIndex(path, table, "BeaconSaved", "telemetryVersion");
            indicesMap.put("telemetryVersion", this.telemetryVersionIndex);
            this.batteryMilliVoltsIndex = getValidColumnIndex(path, table, "BeaconSaved", "batteryMilliVolts");
            indicesMap.put("batteryMilliVolts", this.batteryMilliVoltsIndex);
            this.temperatureIndex = getValidColumnIndex(path, table, "BeaconSaved", "temperature");
            indicesMap.put("temperature", this.temperatureIndex);
            this.pduCountIndex = getValidColumnIndex(path, table, "BeaconSaved", "pduCount");
            indicesMap.put("pduCount", this.pduCountIndex);
            this.uptimeIndex = getValidColumnIndex(path, table, "BeaconSaved", "uptime");
            indicesMap.put("uptime", this.uptimeIndex);

            setIndicesMap(indicesMap);
        }

        @Override
        public final void copyColumnInfoFrom(ColumnInfo other) {
            final BeaconSavedColumnInfo otherInfo = (BeaconSavedColumnInfo) other;
            this.hashcodeIndex = otherInfo.hashcodeIndex;
            this.beaconTypeIndex = otherInfo.beaconTypeIndex;
            this.beaconAddressIndex = otherInfo.beaconAddressIndex;
            this.UUIDIndex = otherInfo.UUIDIndex;
            this.MajorIndex = otherInfo.MajorIndex;
            this.MinorIndex = otherInfo.MinorIndex;
            this.txPowerIndex = otherInfo.txPowerIndex;
            this.RSSIIndex = otherInfo.RSSIIndex;
            this.distanceIndex = otherInfo.distanceIndex;
            this.lastSeenIndex = otherInfo.lastSeenIndex;
            this.lastMinuteSeenIndex = otherInfo.lastMinuteSeenIndex;
            this.manufacturerIndex = otherInfo.manufacturerIndex;
            this.URLIndex = otherInfo.URLIndex;
            this.namespaceIdIndex = otherInfo.namespaceIdIndex;
            this.instanceIdIndex = otherInfo.instanceIdIndex;
            this.hasTelemetryDataIndex = otherInfo.hasTelemetryDataIndex;
            this.telemetryVersionIndex = otherInfo.telemetryVersionIndex;
            this.batteryMilliVoltsIndex = otherInfo.batteryMilliVoltsIndex;
            this.temperatureIndex = otherInfo.temperatureIndex;
            this.pduCountIndex = otherInfo.pduCountIndex;
            this.uptimeIndex = otherInfo.uptimeIndex;

            setIndicesMap(otherInfo.getIndicesMap());
        }

        @Override
        public final BeaconSavedColumnInfo clone() {
            return (BeaconSavedColumnInfo) super.clone();
        }

    }
    private BeaconSavedColumnInfo columnInfo;
    private ProxyState proxyState;
    private static final List<String> FIELD_NAMES;
    static {
        List<String> fieldNames = new ArrayList<String>();
        fieldNames.add("hashcode");
        fieldNames.add("beaconType");
        fieldNames.add("beaconAddress");
        fieldNames.add("UUID");
        fieldNames.add("Major");
        fieldNames.add("Minor");
        fieldNames.add("txPower");
        fieldNames.add("RSSI");
        fieldNames.add("distance");
        fieldNames.add("lastSeen");
        fieldNames.add("lastMinuteSeen");
        fieldNames.add("manufacturer");
        fieldNames.add("URL");
        fieldNames.add("namespaceId");
        fieldNames.add("instanceId");
        fieldNames.add("hasTelemetryData");
        fieldNames.add("telemetryVersion");
        fieldNames.add("batteryMilliVolts");
        fieldNames.add("temperature");
        fieldNames.add("pduCount");
        fieldNames.add("uptime");
        FIELD_NAMES = Collections.unmodifiableList(fieldNames);
    }

    BeaconSavedRealmProxy() {
        if (proxyState == null) {
            injectObjectContext();
        }
        proxyState.setConstructionFinished();
    }

    private void injectObjectContext() {
        final BaseRealm.RealmObjectContext context = BaseRealm.objectContext.get();
        this.columnInfo = (BeaconSavedColumnInfo) context.getColumnInfo();
        this.proxyState = new ProxyState(com.bridou_n.beaconscanner.models.BeaconSaved.class, this);
        proxyState.setRealm$realm(context.getRealm());
        proxyState.setRow$realm(context.getRow());
        proxyState.setAcceptDefaultValue$realm(context.getAcceptDefaultValue());
        proxyState.setExcludeFields$realm(context.getExcludeFields());
    }

    @SuppressWarnings("cast")
    public int realmGet$hashcode() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (int) proxyState.getRow$realm().getLong(columnInfo.hashcodeIndex);
    }

    public void realmSet$hashcode(int value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            // default value of the primary key is always ignored.
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        throw new io.realm.exceptions.RealmException("Primary key field 'hashcode' cannot be changed after object was created.");
    }

    @SuppressWarnings("cast")
    public int realmGet$beaconType() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (int) proxyState.getRow$realm().getLong(columnInfo.beaconTypeIndex);
    }

    public void realmSet$beaconType(int value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            row.getTable().setLong(columnInfo.beaconTypeIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        proxyState.getRow$realm().setLong(columnInfo.beaconTypeIndex, value);
    }

    @SuppressWarnings("cast")
    public String realmGet$beaconAddress() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (java.lang.String) proxyState.getRow$realm().getString(columnInfo.beaconAddressIndex);
    }

    public void realmSet$beaconAddress(String value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            if (value == null) {
                row.getTable().setNull(columnInfo.beaconAddressIndex, row.getIndex(), true);
                return;
            }
            row.getTable().setString(columnInfo.beaconAddressIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        if (value == null) {
            proxyState.getRow$realm().setNull(columnInfo.beaconAddressIndex);
            return;
        }
        proxyState.getRow$realm().setString(columnInfo.beaconAddressIndex, value);
    }

    @SuppressWarnings("cast")
    public String realmGet$UUID() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (java.lang.String) proxyState.getRow$realm().getString(columnInfo.UUIDIndex);
    }

    public void realmSet$UUID(String value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            if (value == null) {
                row.getTable().setNull(columnInfo.UUIDIndex, row.getIndex(), true);
                return;
            }
            row.getTable().setString(columnInfo.UUIDIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        if (value == null) {
            proxyState.getRow$realm().setNull(columnInfo.UUIDIndex);
            return;
        }
        proxyState.getRow$realm().setString(columnInfo.UUIDIndex, value);
    }

    @SuppressWarnings("cast")
    public String realmGet$Major() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (java.lang.String) proxyState.getRow$realm().getString(columnInfo.MajorIndex);
    }

    public void realmSet$Major(String value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            if (value == null) {
                row.getTable().setNull(columnInfo.MajorIndex, row.getIndex(), true);
                return;
            }
            row.getTable().setString(columnInfo.MajorIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        if (value == null) {
            proxyState.getRow$realm().setNull(columnInfo.MajorIndex);
            return;
        }
        proxyState.getRow$realm().setString(columnInfo.MajorIndex, value);
    }

    @SuppressWarnings("cast")
    public String realmGet$Minor() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (java.lang.String) proxyState.getRow$realm().getString(columnInfo.MinorIndex);
    }

    public void realmSet$Minor(String value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            if (value == null) {
                row.getTable().setNull(columnInfo.MinorIndex, row.getIndex(), true);
                return;
            }
            row.getTable().setString(columnInfo.MinorIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        if (value == null) {
            proxyState.getRow$realm().setNull(columnInfo.MinorIndex);
            return;
        }
        proxyState.getRow$realm().setString(columnInfo.MinorIndex, value);
    }

    @SuppressWarnings("cast")
    public int realmGet$txPower() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (int) proxyState.getRow$realm().getLong(columnInfo.txPowerIndex);
    }

    public void realmSet$txPower(int value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            row.getTable().setLong(columnInfo.txPowerIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        proxyState.getRow$realm().setLong(columnInfo.txPowerIndex, value);
    }

    @SuppressWarnings("cast")
    public int realmGet$RSSI() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (int) proxyState.getRow$realm().getLong(columnInfo.RSSIIndex);
    }

    public void realmSet$RSSI(int value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            row.getTable().setLong(columnInfo.RSSIIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        proxyState.getRow$realm().setLong(columnInfo.RSSIIndex, value);
    }

    @SuppressWarnings("cast")
    public double realmGet$distance() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (double) proxyState.getRow$realm().getDouble(columnInfo.distanceIndex);
    }

    public void realmSet$distance(double value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            row.getTable().setDouble(columnInfo.distanceIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        proxyState.getRow$realm().setDouble(columnInfo.distanceIndex, value);
    }

    @SuppressWarnings("cast")
    public Date realmGet$lastSeen() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        if (proxyState.getRow$realm().isNull(columnInfo.lastSeenIndex)) {
            return null;
        }
        return (java.util.Date) proxyState.getRow$realm().getDate(columnInfo.lastSeenIndex);
    }

    public void realmSet$lastSeen(Date value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            if (value == null) {
                row.getTable().setNull(columnInfo.lastSeenIndex, row.getIndex(), true);
                return;
            }
            row.getTable().setDate(columnInfo.lastSeenIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        if (value == null) {
            proxyState.getRow$realm().setNull(columnInfo.lastSeenIndex);
            return;
        }
        proxyState.getRow$realm().setDate(columnInfo.lastSeenIndex, value);
    }

    @SuppressWarnings("cast")
    public long realmGet$lastMinuteSeen() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (long) proxyState.getRow$realm().getLong(columnInfo.lastMinuteSeenIndex);
    }

    public void realmSet$lastMinuteSeen(long value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            row.getTable().setLong(columnInfo.lastMinuteSeenIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        proxyState.getRow$realm().setLong(columnInfo.lastMinuteSeenIndex, value);
    }

    @SuppressWarnings("cast")
    public int realmGet$manufacturer() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (int) proxyState.getRow$realm().getLong(columnInfo.manufacturerIndex);
    }

    public void realmSet$manufacturer(int value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            row.getTable().setLong(columnInfo.manufacturerIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        proxyState.getRow$realm().setLong(columnInfo.manufacturerIndex, value);
    }

    @SuppressWarnings("cast")
    public String realmGet$URL() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (java.lang.String) proxyState.getRow$realm().getString(columnInfo.URLIndex);
    }

    public void realmSet$URL(String value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            if (value == null) {
                row.getTable().setNull(columnInfo.URLIndex, row.getIndex(), true);
                return;
            }
            row.getTable().setString(columnInfo.URLIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        if (value == null) {
            proxyState.getRow$realm().setNull(columnInfo.URLIndex);
            return;
        }
        proxyState.getRow$realm().setString(columnInfo.URLIndex, value);
    }

    @SuppressWarnings("cast")
    public String realmGet$namespaceId() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (java.lang.String) proxyState.getRow$realm().getString(columnInfo.namespaceIdIndex);
    }

    public void realmSet$namespaceId(String value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            if (value == null) {
                row.getTable().setNull(columnInfo.namespaceIdIndex, row.getIndex(), true);
                return;
            }
            row.getTable().setString(columnInfo.namespaceIdIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        if (value == null) {
            proxyState.getRow$realm().setNull(columnInfo.namespaceIdIndex);
            return;
        }
        proxyState.getRow$realm().setString(columnInfo.namespaceIdIndex, value);
    }

    @SuppressWarnings("cast")
    public String realmGet$instanceId() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (java.lang.String) proxyState.getRow$realm().getString(columnInfo.instanceIdIndex);
    }

    public void realmSet$instanceId(String value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            if (value == null) {
                row.getTable().setNull(columnInfo.instanceIdIndex, row.getIndex(), true);
                return;
            }
            row.getTable().setString(columnInfo.instanceIdIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        if (value == null) {
            proxyState.getRow$realm().setNull(columnInfo.instanceIdIndex);
            return;
        }
        proxyState.getRow$realm().setString(columnInfo.instanceIdIndex, value);
    }

    @SuppressWarnings("cast")
    public boolean realmGet$hasTelemetryData() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (boolean) proxyState.getRow$realm().getBoolean(columnInfo.hasTelemetryDataIndex);
    }

    public void realmSet$hasTelemetryData(boolean value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            row.getTable().setBoolean(columnInfo.hasTelemetryDataIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        proxyState.getRow$realm().setBoolean(columnInfo.hasTelemetryDataIndex, value);
    }

    @SuppressWarnings("cast")
    public long realmGet$telemetryVersion() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (long) proxyState.getRow$realm().getLong(columnInfo.telemetryVersionIndex);
    }

    public void realmSet$telemetryVersion(long value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            row.getTable().setLong(columnInfo.telemetryVersionIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        proxyState.getRow$realm().setLong(columnInfo.telemetryVersionIndex, value);
    }

    @SuppressWarnings("cast")
    public long realmGet$batteryMilliVolts() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (long) proxyState.getRow$realm().getLong(columnInfo.batteryMilliVoltsIndex);
    }

    public void realmSet$batteryMilliVolts(long value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            row.getTable().setLong(columnInfo.batteryMilliVoltsIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        proxyState.getRow$realm().setLong(columnInfo.batteryMilliVoltsIndex, value);
    }

    @SuppressWarnings("cast")
    public long realmGet$temperature() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (long) proxyState.getRow$realm().getLong(columnInfo.temperatureIndex);
    }

    public void realmSet$temperature(long value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            row.getTable().setLong(columnInfo.temperatureIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        proxyState.getRow$realm().setLong(columnInfo.temperatureIndex, value);
    }

    @SuppressWarnings("cast")
    public long realmGet$pduCount() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (long) proxyState.getRow$realm().getLong(columnInfo.pduCountIndex);
    }

    public void realmSet$pduCount(long value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            row.getTable().setLong(columnInfo.pduCountIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        proxyState.getRow$realm().setLong(columnInfo.pduCountIndex, value);
    }

    @SuppressWarnings("cast")
    public long realmGet$uptime() {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        proxyState.getRealm$realm().checkIfValid();
        return (long) proxyState.getRow$realm().getLong(columnInfo.uptimeIndex);
    }

    public void realmSet$uptime(long value) {
        if (proxyState == null) {
            // Called from model's constructor. Inject context.
            injectObjectContext();
        }

        if (proxyState.isUnderConstruction()) {
            if (!proxyState.getAcceptDefaultValue$realm()) {
                return;
            }
            final Row row = proxyState.getRow$realm();
            row.getTable().setLong(columnInfo.uptimeIndex, row.getIndex(), value, true);
            return;
        }

        proxyState.getRealm$realm().checkIfValid();
        proxyState.getRow$realm().setLong(columnInfo.uptimeIndex, value);
    }

    public static RealmObjectSchema createRealmObjectSchema(RealmSchema realmSchema) {
        if (!realmSchema.contains("BeaconSaved")) {
            RealmObjectSchema realmObjectSchema = realmSchema.create("BeaconSaved");
            realmObjectSchema.add(new Property("hashcode", RealmFieldType.INTEGER, Property.PRIMARY_KEY, Property.INDEXED, Property.REQUIRED));
            realmObjectSchema.add(new Property("beaconType", RealmFieldType.INTEGER, !Property.PRIMARY_KEY, !Property.INDEXED, Property.REQUIRED));
            realmObjectSchema.add(new Property("beaconAddress", RealmFieldType.STRING, !Property.PRIMARY_KEY, !Property.INDEXED, !Property.REQUIRED));
            realmObjectSchema.add(new Property("UUID", RealmFieldType.STRING, !Property.PRIMARY_KEY, !Property.INDEXED, !Property.REQUIRED));
            realmObjectSchema.add(new Property("Major", RealmFieldType.STRING, !Property.PRIMARY_KEY, !Property.INDEXED, !Property.REQUIRED));
            realmObjectSchema.add(new Property("Minor", RealmFieldType.STRING, !Property.PRIMARY_KEY, !Property.INDEXED, !Property.REQUIRED));
            realmObjectSchema.add(new Property("txPower", RealmFieldType.INTEGER, !Property.PRIMARY_KEY, !Property.INDEXED, Property.REQUIRED));
            realmObjectSchema.add(new Property("RSSI", RealmFieldType.INTEGER, !Property.PRIMARY_KEY, !Property.INDEXED, Property.REQUIRED));
            realmObjectSchema.add(new Property("distance", RealmFieldType.DOUBLE, !Property.PRIMARY_KEY, !Property.INDEXED, Property.REQUIRED));
            realmObjectSchema.add(new Property("lastSeen", RealmFieldType.DATE, !Property.PRIMARY_KEY, !Property.INDEXED, !Property.REQUIRED));
            realmObjectSchema.add(new Property("lastMinuteSeen", RealmFieldType.INTEGER, !Property.PRIMARY_KEY, !Property.INDEXED, Property.REQUIRED));
            realmObjectSchema.add(new Property("manufacturer", RealmFieldType.INTEGER, !Property.PRIMARY_KEY, !Property.INDEXED, Property.REQUIRED));
            realmObjectSchema.add(new Property("URL", RealmFieldType.STRING, !Property.PRIMARY_KEY, !Property.INDEXED, !Property.REQUIRED));
            realmObjectSchema.add(new Property("namespaceId", RealmFieldType.STRING, !Property.PRIMARY_KEY, !Property.INDEXED, !Property.REQUIRED));
            realmObjectSchema.add(new Property("instanceId", RealmFieldType.STRING, !Property.PRIMARY_KEY, !Property.INDEXED, !Property.REQUIRED));
            realmObjectSchema.add(new Property("hasTelemetryData", RealmFieldType.BOOLEAN, !Property.PRIMARY_KEY, !Property.INDEXED, Property.REQUIRED));
            realmObjectSchema.add(new Property("telemetryVersion", RealmFieldType.INTEGER, !Property.PRIMARY_KEY, !Property.INDEXED, Property.REQUIRED));
            realmObjectSchema.add(new Property("batteryMilliVolts", RealmFieldType.INTEGER, !Property.PRIMARY_KEY, !Property.INDEXED, Property.REQUIRED));
            realmObjectSchema.add(new Property("temperature", RealmFieldType.INTEGER, !Property.PRIMARY_KEY, !Property.INDEXED, Property.REQUIRED));
            realmObjectSchema.add(new Property("pduCount", RealmFieldType.INTEGER, !Property.PRIMARY_KEY, !Property.INDEXED, Property.REQUIRED));
            realmObjectSchema.add(new Property("uptime", RealmFieldType.INTEGER, !Property.PRIMARY_KEY, !Property.INDEXED, Property.REQUIRED));
            return realmObjectSchema;
        }
        return realmSchema.get("BeaconSaved");
    }

    public static Table initTable(SharedRealm sharedRealm) {
        if (!sharedRealm.hasTable("class_BeaconSaved")) {
            Table table = sharedRealm.getTable("class_BeaconSaved");
            table.addColumn(RealmFieldType.INTEGER, "hashcode", Table.NOT_NULLABLE);
            table.addColumn(RealmFieldType.INTEGER, "beaconType", Table.NOT_NULLABLE);
            table.addColumn(RealmFieldType.STRING, "beaconAddress", Table.NULLABLE);
            table.addColumn(RealmFieldType.STRING, "UUID", Table.NULLABLE);
            table.addColumn(RealmFieldType.STRING, "Major", Table.NULLABLE);
            table.addColumn(RealmFieldType.STRING, "Minor", Table.NULLABLE);
            table.addColumn(RealmFieldType.INTEGER, "txPower", Table.NOT_NULLABLE);
            table.addColumn(RealmFieldType.INTEGER, "RSSI", Table.NOT_NULLABLE);
            table.addColumn(RealmFieldType.DOUBLE, "distance", Table.NOT_NULLABLE);
            table.addColumn(RealmFieldType.DATE, "lastSeen", Table.NULLABLE);
            table.addColumn(RealmFieldType.INTEGER, "lastMinuteSeen", Table.NOT_NULLABLE);
            table.addColumn(RealmFieldType.INTEGER, "manufacturer", Table.NOT_NULLABLE);
            table.addColumn(RealmFieldType.STRING, "URL", Table.NULLABLE);
            table.addColumn(RealmFieldType.STRING, "namespaceId", Table.NULLABLE);
            table.addColumn(RealmFieldType.STRING, "instanceId", Table.NULLABLE);
            table.addColumn(RealmFieldType.BOOLEAN, "hasTelemetryData", Table.NOT_NULLABLE);
            table.addColumn(RealmFieldType.INTEGER, "telemetryVersion", Table.NOT_NULLABLE);
            table.addColumn(RealmFieldType.INTEGER, "batteryMilliVolts", Table.NOT_NULLABLE);
            table.addColumn(RealmFieldType.INTEGER, "temperature", Table.NOT_NULLABLE);
            table.addColumn(RealmFieldType.INTEGER, "pduCount", Table.NOT_NULLABLE);
            table.addColumn(RealmFieldType.INTEGER, "uptime", Table.NOT_NULLABLE);
            table.addSearchIndex(table.getColumnIndex("hashcode"));
            table.setPrimaryKey("hashcode");
            return table;
        }
        return sharedRealm.getTable("class_BeaconSaved");
    }

    public static BeaconSavedColumnInfo validateTable(SharedRealm sharedRealm, boolean allowExtraColumns) {
        if (sharedRealm.hasTable("class_BeaconSaved")) {
            Table table = sharedRealm.getTable("class_BeaconSaved");
            final long columnCount = table.getColumnCount();
            if (columnCount != 21) {
                if (columnCount < 21) {
                    throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field count is less than expected - expected 21 but was " + columnCount);
                }
                if (allowExtraColumns) {
                    RealmLog.debug("Field count is more than expected - expected 21 but was %1$d", columnCount);
                } else {
                    throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field count is more than expected - expected 21 but was " + columnCount);
                }
            }
            Map<String, RealmFieldType> columnTypes = new HashMap<String, RealmFieldType>();
            for (long i = 0; i < 21; i++) {
                columnTypes.put(table.getColumnName(i), table.getColumnType(i));
            }

            final BeaconSavedColumnInfo columnInfo = new BeaconSavedColumnInfo(sharedRealm.getPath(), table);

            if (!columnTypes.containsKey("hashcode")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'hashcode' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("hashcode") != RealmFieldType.INTEGER) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'int' for field 'hashcode' in existing Realm file.");
            }
            if (table.isColumnNullable(columnInfo.hashcodeIndex) && table.findFirstNull(columnInfo.hashcodeIndex) != TableOrView.NO_MATCH) {
                throw new IllegalStateException("Cannot migrate an object with null value in field 'hashcode'. Either maintain the same type for primary key field 'hashcode', or remove the object with null value before migration.");
            }
            if (table.getPrimaryKey() != table.getColumnIndex("hashcode")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Primary key not defined for field 'hashcode' in existing Realm file. Add @PrimaryKey.");
            }
            if (!table.hasSearchIndex(table.getColumnIndex("hashcode"))) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Index not defined for field 'hashcode' in existing Realm file. Either set @Index or migrate using io.realm.internal.Table.removeSearchIndex().");
            }
            if (!columnTypes.containsKey("beaconType")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'beaconType' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("beaconType") != RealmFieldType.INTEGER) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'int' for field 'beaconType' in existing Realm file.");
            }
            if (table.isColumnNullable(columnInfo.beaconTypeIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'beaconType' does support null values in the existing Realm file. Use corresponding boxed type for field 'beaconType' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("beaconAddress")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'beaconAddress' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("beaconAddress") != RealmFieldType.STRING) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'String' for field 'beaconAddress' in existing Realm file.");
            }
            if (!table.isColumnNullable(columnInfo.beaconAddressIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'beaconAddress' is required. Either set @Required to field 'beaconAddress' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("UUID")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'UUID' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("UUID") != RealmFieldType.STRING) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'String' for field 'UUID' in existing Realm file.");
            }
            if (!table.isColumnNullable(columnInfo.UUIDIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'UUID' is required. Either set @Required to field 'UUID' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("Major")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'Major' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("Major") != RealmFieldType.STRING) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'String' for field 'Major' in existing Realm file.");
            }
            if (!table.isColumnNullable(columnInfo.MajorIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'Major' is required. Either set @Required to field 'Major' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("Minor")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'Minor' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("Minor") != RealmFieldType.STRING) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'String' for field 'Minor' in existing Realm file.");
            }
            if (!table.isColumnNullable(columnInfo.MinorIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'Minor' is required. Either set @Required to field 'Minor' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("txPower")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'txPower' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("txPower") != RealmFieldType.INTEGER) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'int' for field 'txPower' in existing Realm file.");
            }
            if (table.isColumnNullable(columnInfo.txPowerIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'txPower' does support null values in the existing Realm file. Use corresponding boxed type for field 'txPower' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("RSSI")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'RSSI' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("RSSI") != RealmFieldType.INTEGER) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'int' for field 'RSSI' in existing Realm file.");
            }
            if (table.isColumnNullable(columnInfo.RSSIIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'RSSI' does support null values in the existing Realm file. Use corresponding boxed type for field 'RSSI' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("distance")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'distance' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("distance") != RealmFieldType.DOUBLE) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'double' for field 'distance' in existing Realm file.");
            }
            if (table.isColumnNullable(columnInfo.distanceIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'distance' does support null values in the existing Realm file. Use corresponding boxed type for field 'distance' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("lastSeen")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'lastSeen' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("lastSeen") != RealmFieldType.DATE) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'Date' for field 'lastSeen' in existing Realm file.");
            }
            if (!table.isColumnNullable(columnInfo.lastSeenIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'lastSeen' is required. Either set @Required to field 'lastSeen' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("lastMinuteSeen")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'lastMinuteSeen' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("lastMinuteSeen") != RealmFieldType.INTEGER) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'long' for field 'lastMinuteSeen' in existing Realm file.");
            }
            if (table.isColumnNullable(columnInfo.lastMinuteSeenIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'lastMinuteSeen' does support null values in the existing Realm file. Use corresponding boxed type for field 'lastMinuteSeen' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("manufacturer")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'manufacturer' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("manufacturer") != RealmFieldType.INTEGER) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'int' for field 'manufacturer' in existing Realm file.");
            }
            if (table.isColumnNullable(columnInfo.manufacturerIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'manufacturer' does support null values in the existing Realm file. Use corresponding boxed type for field 'manufacturer' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("URL")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'URL' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("URL") != RealmFieldType.STRING) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'String' for field 'URL' in existing Realm file.");
            }
            if (!table.isColumnNullable(columnInfo.URLIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'URL' is required. Either set @Required to field 'URL' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("namespaceId")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'namespaceId' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("namespaceId") != RealmFieldType.STRING) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'String' for field 'namespaceId' in existing Realm file.");
            }
            if (!table.isColumnNullable(columnInfo.namespaceIdIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'namespaceId' is required. Either set @Required to field 'namespaceId' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("instanceId")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'instanceId' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("instanceId") != RealmFieldType.STRING) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'String' for field 'instanceId' in existing Realm file.");
            }
            if (!table.isColumnNullable(columnInfo.instanceIdIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'instanceId' is required. Either set @Required to field 'instanceId' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("hasTelemetryData")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'hasTelemetryData' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("hasTelemetryData") != RealmFieldType.BOOLEAN) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'boolean' for field 'hasTelemetryData' in existing Realm file.");
            }
            if (table.isColumnNullable(columnInfo.hasTelemetryDataIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'hasTelemetryData' does support null values in the existing Realm file. Use corresponding boxed type for field 'hasTelemetryData' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("telemetryVersion")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'telemetryVersion' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("telemetryVersion") != RealmFieldType.INTEGER) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'long' for field 'telemetryVersion' in existing Realm file.");
            }
            if (table.isColumnNullable(columnInfo.telemetryVersionIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'telemetryVersion' does support null values in the existing Realm file. Use corresponding boxed type for field 'telemetryVersion' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("batteryMilliVolts")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'batteryMilliVolts' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("batteryMilliVolts") != RealmFieldType.INTEGER) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'long' for field 'batteryMilliVolts' in existing Realm file.");
            }
            if (table.isColumnNullable(columnInfo.batteryMilliVoltsIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'batteryMilliVolts' does support null values in the existing Realm file. Use corresponding boxed type for field 'batteryMilliVolts' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("temperature")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'temperature' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("temperature") != RealmFieldType.INTEGER) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'long' for field 'temperature' in existing Realm file.");
            }
            if (table.isColumnNullable(columnInfo.temperatureIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'temperature' does support null values in the existing Realm file. Use corresponding boxed type for field 'temperature' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("pduCount")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'pduCount' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("pduCount") != RealmFieldType.INTEGER) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'long' for field 'pduCount' in existing Realm file.");
            }
            if (table.isColumnNullable(columnInfo.pduCountIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'pduCount' does support null values in the existing Realm file. Use corresponding boxed type for field 'pduCount' or migrate using RealmObjectSchema.setNullable().");
            }
            if (!columnTypes.containsKey("uptime")) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Missing field 'uptime' in existing Realm file. Either remove field or migrate using io.realm.internal.Table.addColumn().");
            }
            if (columnTypes.get("uptime") != RealmFieldType.INTEGER) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Invalid type 'long' for field 'uptime' in existing Realm file.");
            }
            if (table.isColumnNullable(columnInfo.uptimeIndex)) {
                throw new RealmMigrationNeededException(sharedRealm.getPath(), "Field 'uptime' does support null values in the existing Realm file. Use corresponding boxed type for field 'uptime' or migrate using RealmObjectSchema.setNullable().");
            }
            return columnInfo;
        } else {
            throw new RealmMigrationNeededException(sharedRealm.getPath(), "The 'BeaconSaved' class is missing from the schema for this Realm.");
        }
    }

    public static String getTableName() {
        return "class_BeaconSaved";
    }

    public static List<String> getFieldNames() {
        return FIELD_NAMES;
    }

    @SuppressWarnings("cast")
    public static com.bridou_n.beaconscanner.models.BeaconSaved createOrUpdateUsingJsonObject(Realm realm, JSONObject json, boolean update)
        throws JSONException {
        final List<String> excludeFields = Collections.<String> emptyList();
        com.bridou_n.beaconscanner.models.BeaconSaved obj = null;
        if (update) {
            Table table = realm.getTable(com.bridou_n.beaconscanner.models.BeaconSaved.class);
            long pkColumnIndex = table.getPrimaryKey();
            long rowIndex = TableOrView.NO_MATCH;
            if (!json.isNull("hashcode")) {
                rowIndex = table.findFirstLong(pkColumnIndex, json.getLong("hashcode"));
            }
            if (rowIndex != TableOrView.NO_MATCH) {
                final BaseRealm.RealmObjectContext objectContext = BaseRealm.objectContext.get();
                try {
                    objectContext.set(realm, table.getUncheckedRow(rowIndex), realm.schema.getColumnInfo(com.bridou_n.beaconscanner.models.BeaconSaved.class), false, Collections.<String> emptyList());
                    obj = new io.realm.BeaconSavedRealmProxy();
                } finally {
                    objectContext.clear();
                }
            }
        }
        if (obj == null) {
            if (json.has("hashcode")) {
                if (json.isNull("hashcode")) {
                    obj = (io.realm.BeaconSavedRealmProxy) realm.createObjectInternal(com.bridou_n.beaconscanner.models.BeaconSaved.class, null, true, excludeFields);
                } else {
                    obj = (io.realm.BeaconSavedRealmProxy) realm.createObjectInternal(com.bridou_n.beaconscanner.models.BeaconSaved.class, json.getInt("hashcode"), true, excludeFields);
                }
            } else {
                throw new IllegalArgumentException("JSON object doesn't have the primary key field 'hashcode'.");
            }
        }
        if (json.has("beaconType")) {
            if (json.isNull("beaconType")) {
                throw new IllegalArgumentException("Trying to set non-nullable field 'beaconType' to null.");
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$beaconType((int) json.getInt("beaconType"));
            }
        }
        if (json.has("beaconAddress")) {
            if (json.isNull("beaconAddress")) {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$beaconAddress(null);
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$beaconAddress((String) json.getString("beaconAddress"));
            }
        }
        if (json.has("UUID")) {
            if (json.isNull("UUID")) {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$UUID(null);
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$UUID((String) json.getString("UUID"));
            }
        }
        if (json.has("Major")) {
            if (json.isNull("Major")) {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$Major(null);
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$Major((String) json.getString("Major"));
            }
        }
        if (json.has("Minor")) {
            if (json.isNull("Minor")) {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$Minor(null);
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$Minor((String) json.getString("Minor"));
            }
        }
        if (json.has("txPower")) {
            if (json.isNull("txPower")) {
                throw new IllegalArgumentException("Trying to set non-nullable field 'txPower' to null.");
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$txPower((int) json.getInt("txPower"));
            }
        }
        if (json.has("RSSI")) {
            if (json.isNull("RSSI")) {
                throw new IllegalArgumentException("Trying to set non-nullable field 'RSSI' to null.");
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$RSSI((int) json.getInt("RSSI"));
            }
        }
        if (json.has("distance")) {
            if (json.isNull("distance")) {
                throw new IllegalArgumentException("Trying to set non-nullable field 'distance' to null.");
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$distance((double) json.getDouble("distance"));
            }
        }
        if (json.has("lastSeen")) {
            if (json.isNull("lastSeen")) {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$lastSeen(null);
            } else {
                Object timestamp = json.get("lastSeen");
                if (timestamp instanceof String) {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$lastSeen(JsonUtils.stringToDate((String) timestamp));
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$lastSeen(new Date(json.getLong("lastSeen")));
                }
            }
        }
        if (json.has("lastMinuteSeen")) {
            if (json.isNull("lastMinuteSeen")) {
                throw new IllegalArgumentException("Trying to set non-nullable field 'lastMinuteSeen' to null.");
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$lastMinuteSeen((long) json.getLong("lastMinuteSeen"));
            }
        }
        if (json.has("manufacturer")) {
            if (json.isNull("manufacturer")) {
                throw new IllegalArgumentException("Trying to set non-nullable field 'manufacturer' to null.");
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$manufacturer((int) json.getInt("manufacturer"));
            }
        }
        if (json.has("URL")) {
            if (json.isNull("URL")) {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$URL(null);
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$URL((String) json.getString("URL"));
            }
        }
        if (json.has("namespaceId")) {
            if (json.isNull("namespaceId")) {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$namespaceId(null);
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$namespaceId((String) json.getString("namespaceId"));
            }
        }
        if (json.has("instanceId")) {
            if (json.isNull("instanceId")) {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$instanceId(null);
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$instanceId((String) json.getString("instanceId"));
            }
        }
        if (json.has("hasTelemetryData")) {
            if (json.isNull("hasTelemetryData")) {
                throw new IllegalArgumentException("Trying to set non-nullable field 'hasTelemetryData' to null.");
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$hasTelemetryData((boolean) json.getBoolean("hasTelemetryData"));
            }
        }
        if (json.has("telemetryVersion")) {
            if (json.isNull("telemetryVersion")) {
                throw new IllegalArgumentException("Trying to set non-nullable field 'telemetryVersion' to null.");
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$telemetryVersion((long) json.getLong("telemetryVersion"));
            }
        }
        if (json.has("batteryMilliVolts")) {
            if (json.isNull("batteryMilliVolts")) {
                throw new IllegalArgumentException("Trying to set non-nullable field 'batteryMilliVolts' to null.");
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$batteryMilliVolts((long) json.getLong("batteryMilliVolts"));
            }
        }
        if (json.has("temperature")) {
            if (json.isNull("temperature")) {
                throw new IllegalArgumentException("Trying to set non-nullable field 'temperature' to null.");
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$temperature((long) json.getLong("temperature"));
            }
        }
        if (json.has("pduCount")) {
            if (json.isNull("pduCount")) {
                throw new IllegalArgumentException("Trying to set non-nullable field 'pduCount' to null.");
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$pduCount((long) json.getLong("pduCount"));
            }
        }
        if (json.has("uptime")) {
            if (json.isNull("uptime")) {
                throw new IllegalArgumentException("Trying to set non-nullable field 'uptime' to null.");
            } else {
                ((BeaconSavedRealmProxyInterface) obj).realmSet$uptime((long) json.getLong("uptime"));
            }
        }
        return obj;
    }

    @SuppressWarnings("cast")
    @TargetApi(Build.VERSION_CODES.HONEYCOMB)
    public static com.bridou_n.beaconscanner.models.BeaconSaved createUsingJsonStream(Realm realm, JsonReader reader)
        throws IOException {
        boolean jsonHasPrimaryKey = false;
        com.bridou_n.beaconscanner.models.BeaconSaved obj = new com.bridou_n.beaconscanner.models.BeaconSaved();
        reader.beginObject();
        while (reader.hasNext()) {
            String name = reader.nextName();
            if (name.equals("hashcode")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    throw new IllegalArgumentException("Trying to set non-nullable field 'hashcode' to null.");
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$hashcode((int) reader.nextInt());
                }
                jsonHasPrimaryKey = true;
            } else if (name.equals("beaconType")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    throw new IllegalArgumentException("Trying to set non-nullable field 'beaconType' to null.");
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$beaconType((int) reader.nextInt());
                }
            } else if (name.equals("beaconAddress")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$beaconAddress(null);
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$beaconAddress((String) reader.nextString());
                }
            } else if (name.equals("UUID")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$UUID(null);
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$UUID((String) reader.nextString());
                }
            } else if (name.equals("Major")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$Major(null);
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$Major((String) reader.nextString());
                }
            } else if (name.equals("Minor")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$Minor(null);
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$Minor((String) reader.nextString());
                }
            } else if (name.equals("txPower")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    throw new IllegalArgumentException("Trying to set non-nullable field 'txPower' to null.");
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$txPower((int) reader.nextInt());
                }
            } else if (name.equals("RSSI")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    throw new IllegalArgumentException("Trying to set non-nullable field 'RSSI' to null.");
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$RSSI((int) reader.nextInt());
                }
            } else if (name.equals("distance")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    throw new IllegalArgumentException("Trying to set non-nullable field 'distance' to null.");
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$distance((double) reader.nextDouble());
                }
            } else if (name.equals("lastSeen")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$lastSeen(null);
                } else if (reader.peek() == JsonToken.NUMBER) {
                    long timestamp = reader.nextLong();
                    if (timestamp > -1) {
                        ((BeaconSavedRealmProxyInterface) obj).realmSet$lastSeen(new Date(timestamp));
                    }
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$lastSeen(JsonUtils.stringToDate(reader.nextString()));
                }
            } else if (name.equals("lastMinuteSeen")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    throw new IllegalArgumentException("Trying to set non-nullable field 'lastMinuteSeen' to null.");
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$lastMinuteSeen((long) reader.nextLong());
                }
            } else if (name.equals("manufacturer")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    throw new IllegalArgumentException("Trying to set non-nullable field 'manufacturer' to null.");
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$manufacturer((int) reader.nextInt());
                }
            } else if (name.equals("URL")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$URL(null);
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$URL((String) reader.nextString());
                }
            } else if (name.equals("namespaceId")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$namespaceId(null);
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$namespaceId((String) reader.nextString());
                }
            } else if (name.equals("instanceId")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$instanceId(null);
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$instanceId((String) reader.nextString());
                }
            } else if (name.equals("hasTelemetryData")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    throw new IllegalArgumentException("Trying to set non-nullable field 'hasTelemetryData' to null.");
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$hasTelemetryData((boolean) reader.nextBoolean());
                }
            } else if (name.equals("telemetryVersion")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    throw new IllegalArgumentException("Trying to set non-nullable field 'telemetryVersion' to null.");
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$telemetryVersion((long) reader.nextLong());
                }
            } else if (name.equals("batteryMilliVolts")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    throw new IllegalArgumentException("Trying to set non-nullable field 'batteryMilliVolts' to null.");
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$batteryMilliVolts((long) reader.nextLong());
                }
            } else if (name.equals("temperature")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    throw new IllegalArgumentException("Trying to set non-nullable field 'temperature' to null.");
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$temperature((long) reader.nextLong());
                }
            } else if (name.equals("pduCount")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    throw new IllegalArgumentException("Trying to set non-nullable field 'pduCount' to null.");
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$pduCount((long) reader.nextLong());
                }
            } else if (name.equals("uptime")) {
                if (reader.peek() == JsonToken.NULL) {
                    reader.skipValue();
                    throw new IllegalArgumentException("Trying to set non-nullable field 'uptime' to null.");
                } else {
                    ((BeaconSavedRealmProxyInterface) obj).realmSet$uptime((long) reader.nextLong());
                }
            } else {
                reader.skipValue();
            }
        }
        reader.endObject();
        if (!jsonHasPrimaryKey) {
            throw new IllegalArgumentException("JSON object doesn't have the primary key field 'hashcode'.");
        }
        obj = realm.copyToRealm(obj);
        return obj;
    }

    public static com.bridou_n.beaconscanner.models.BeaconSaved copyOrUpdate(Realm realm, com.bridou_n.beaconscanner.models.BeaconSaved object, boolean update, Map<RealmModel,RealmObjectProxy> cache) {
        if (object instanceof RealmObjectProxy && ((RealmObjectProxy) object).realmGet$proxyState().getRealm$realm() != null && ((RealmObjectProxy) object).realmGet$proxyState().getRealm$realm().threadId != realm.threadId) {
            throw new IllegalArgumentException("Objects which belong to Realm instances in other threads cannot be copied into this Realm instance.");
        }
        if (object instanceof RealmObjectProxy && ((RealmObjectProxy)object).realmGet$proxyState().getRealm$realm() != null && ((RealmObjectProxy)object).realmGet$proxyState().getRealm$realm().getPath().equals(realm.getPath())) {
            return object;
        }
        final BaseRealm.RealmObjectContext objectContext = BaseRealm.objectContext.get();
        RealmObjectProxy cachedRealmObject = cache.get(object);
        if (cachedRealmObject != null) {
            return (com.bridou_n.beaconscanner.models.BeaconSaved) cachedRealmObject;
        } else {
            com.bridou_n.beaconscanner.models.BeaconSaved realmObject = null;
            boolean canUpdate = update;
            if (canUpdate) {
                Table table = realm.getTable(com.bridou_n.beaconscanner.models.BeaconSaved.class);
                long pkColumnIndex = table.getPrimaryKey();
                long rowIndex = table.findFirstLong(pkColumnIndex, ((BeaconSavedRealmProxyInterface) object).realmGet$hashcode());
                if (rowIndex != TableOrView.NO_MATCH) {
                    try {
                        objectContext.set(realm, table.getUncheckedRow(rowIndex), realm.schema.getColumnInfo(com.bridou_n.beaconscanner.models.BeaconSaved.class), false, Collections.<String> emptyList());
                        realmObject = new io.realm.BeaconSavedRealmProxy();
                        cache.put(object, (RealmObjectProxy) realmObject);
                    } finally {
                        objectContext.clear();
                    }
                } else {
                    canUpdate = false;
                }
            }

            if (canUpdate) {
                return update(realm, realmObject, object, cache);
            } else {
                return copy(realm, object, update, cache);
            }
        }
    }

    public static com.bridou_n.beaconscanner.models.BeaconSaved copy(Realm realm, com.bridou_n.beaconscanner.models.BeaconSaved newObject, boolean update, Map<RealmModel,RealmObjectProxy> cache) {
        RealmObjectProxy cachedRealmObject = cache.get(newObject);
        if (cachedRealmObject != null) {
            return (com.bridou_n.beaconscanner.models.BeaconSaved) cachedRealmObject;
        } else {
            // rejecting default values to avoid creating unexpected objects from RealmModel/RealmList fields.
            com.bridou_n.beaconscanner.models.BeaconSaved realmObject = realm.createObjectInternal(com.bridou_n.beaconscanner.models.BeaconSaved.class, ((BeaconSavedRealmProxyInterface) newObject).realmGet$hashcode(), false, Collections.<String>emptyList());
            cache.put(newObject, (RealmObjectProxy) realmObject);
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$beaconType(((BeaconSavedRealmProxyInterface) newObject).realmGet$beaconType());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$beaconAddress(((BeaconSavedRealmProxyInterface) newObject).realmGet$beaconAddress());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$UUID(((BeaconSavedRealmProxyInterface) newObject).realmGet$UUID());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$Major(((BeaconSavedRealmProxyInterface) newObject).realmGet$Major());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$Minor(((BeaconSavedRealmProxyInterface) newObject).realmGet$Minor());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$txPower(((BeaconSavedRealmProxyInterface) newObject).realmGet$txPower());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$RSSI(((BeaconSavedRealmProxyInterface) newObject).realmGet$RSSI());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$distance(((BeaconSavedRealmProxyInterface) newObject).realmGet$distance());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$lastSeen(((BeaconSavedRealmProxyInterface) newObject).realmGet$lastSeen());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$lastMinuteSeen(((BeaconSavedRealmProxyInterface) newObject).realmGet$lastMinuteSeen());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$manufacturer(((BeaconSavedRealmProxyInterface) newObject).realmGet$manufacturer());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$URL(((BeaconSavedRealmProxyInterface) newObject).realmGet$URL());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$namespaceId(((BeaconSavedRealmProxyInterface) newObject).realmGet$namespaceId());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$instanceId(((BeaconSavedRealmProxyInterface) newObject).realmGet$instanceId());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$hasTelemetryData(((BeaconSavedRealmProxyInterface) newObject).realmGet$hasTelemetryData());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$telemetryVersion(((BeaconSavedRealmProxyInterface) newObject).realmGet$telemetryVersion());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$batteryMilliVolts(((BeaconSavedRealmProxyInterface) newObject).realmGet$batteryMilliVolts());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$temperature(((BeaconSavedRealmProxyInterface) newObject).realmGet$temperature());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$pduCount(((BeaconSavedRealmProxyInterface) newObject).realmGet$pduCount());
            ((BeaconSavedRealmProxyInterface) realmObject).realmSet$uptime(((BeaconSavedRealmProxyInterface) newObject).realmGet$uptime());
            return realmObject;
        }
    }

    public static long insert(Realm realm, com.bridou_n.beaconscanner.models.BeaconSaved object, Map<RealmModel,Long> cache) {
        if (object instanceof RealmObjectProxy && ((RealmObjectProxy)object).realmGet$proxyState().getRealm$realm() != null && ((RealmObjectProxy)object).realmGet$proxyState().getRealm$realm().getPath().equals(realm.getPath())) {
            return ((RealmObjectProxy)object).realmGet$proxyState().getRow$realm().getIndex();
        }
        Table table = realm.getTable(com.bridou_n.beaconscanner.models.BeaconSaved.class);
        long tableNativePtr = table.getNativeTablePointer();
        BeaconSavedColumnInfo columnInfo = (BeaconSavedColumnInfo) realm.schema.getColumnInfo(com.bridou_n.beaconscanner.models.BeaconSaved.class);
        long pkColumnIndex = table.getPrimaryKey();
        long rowIndex = TableOrView.NO_MATCH;
        Object primaryKeyValue = ((BeaconSavedRealmProxyInterface) object).realmGet$hashcode();
        if (primaryKeyValue != null) {
            rowIndex = Table.nativeFindFirstInt(tableNativePtr, pkColumnIndex, ((BeaconSavedRealmProxyInterface) object).realmGet$hashcode());
        }
        if (rowIndex == TableOrView.NO_MATCH) {
            rowIndex = table.addEmptyRowWithPrimaryKey(((BeaconSavedRealmProxyInterface) object).realmGet$hashcode(), false);
        } else {
            Table.throwDuplicatePrimaryKeyException(primaryKeyValue);
        }
        cache.put(object, rowIndex);
        Table.nativeSetLong(tableNativePtr, columnInfo.beaconTypeIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$beaconType(), false);
        String realmGet$beaconAddress = ((BeaconSavedRealmProxyInterface)object).realmGet$beaconAddress();
        if (realmGet$beaconAddress != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.beaconAddressIndex, rowIndex, realmGet$beaconAddress, false);
        }
        String realmGet$UUID = ((BeaconSavedRealmProxyInterface)object).realmGet$UUID();
        if (realmGet$UUID != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.UUIDIndex, rowIndex, realmGet$UUID, false);
        }
        String realmGet$Major = ((BeaconSavedRealmProxyInterface)object).realmGet$Major();
        if (realmGet$Major != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.MajorIndex, rowIndex, realmGet$Major, false);
        }
        String realmGet$Minor = ((BeaconSavedRealmProxyInterface)object).realmGet$Minor();
        if (realmGet$Minor != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.MinorIndex, rowIndex, realmGet$Minor, false);
        }
        Table.nativeSetLong(tableNativePtr, columnInfo.txPowerIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$txPower(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.RSSIIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$RSSI(), false);
        Table.nativeSetDouble(tableNativePtr, columnInfo.distanceIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$distance(), false);
        java.util.Date realmGet$lastSeen = ((BeaconSavedRealmProxyInterface)object).realmGet$lastSeen();
        if (realmGet$lastSeen != null) {
            Table.nativeSetTimestamp(tableNativePtr, columnInfo.lastSeenIndex, rowIndex, realmGet$lastSeen.getTime(), false);
        }
        Table.nativeSetLong(tableNativePtr, columnInfo.lastMinuteSeenIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$lastMinuteSeen(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.manufacturerIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$manufacturer(), false);
        String realmGet$URL = ((BeaconSavedRealmProxyInterface)object).realmGet$URL();
        if (realmGet$URL != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.URLIndex, rowIndex, realmGet$URL, false);
        }
        String realmGet$namespaceId = ((BeaconSavedRealmProxyInterface)object).realmGet$namespaceId();
        if (realmGet$namespaceId != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.namespaceIdIndex, rowIndex, realmGet$namespaceId, false);
        }
        String realmGet$instanceId = ((BeaconSavedRealmProxyInterface)object).realmGet$instanceId();
        if (realmGet$instanceId != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.instanceIdIndex, rowIndex, realmGet$instanceId, false);
        }
        Table.nativeSetBoolean(tableNativePtr, columnInfo.hasTelemetryDataIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$hasTelemetryData(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.telemetryVersionIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$telemetryVersion(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.batteryMilliVoltsIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$batteryMilliVolts(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.temperatureIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$temperature(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.pduCountIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$pduCount(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.uptimeIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$uptime(), false);
        return rowIndex;
    }

    public static void insert(Realm realm, Iterator<? extends RealmModel> objects, Map<RealmModel,Long> cache) {
        Table table = realm.getTable(com.bridou_n.beaconscanner.models.BeaconSaved.class);
        long tableNativePtr = table.getNativeTablePointer();
        BeaconSavedColumnInfo columnInfo = (BeaconSavedColumnInfo) realm.schema.getColumnInfo(com.bridou_n.beaconscanner.models.BeaconSaved.class);
        long pkColumnIndex = table.getPrimaryKey();
        com.bridou_n.beaconscanner.models.BeaconSaved object = null;
        while (objects.hasNext()) {
            object = (com.bridou_n.beaconscanner.models.BeaconSaved) objects.next();
            if(!cache.containsKey(object)) {
                if (object instanceof RealmObjectProxy && ((RealmObjectProxy)object).realmGet$proxyState().getRealm$realm() != null && ((RealmObjectProxy)object).realmGet$proxyState().getRealm$realm().getPath().equals(realm.getPath())) {
                    cache.put(object, ((RealmObjectProxy)object).realmGet$proxyState().getRow$realm().getIndex());
                    continue;
                }
                long rowIndex = TableOrView.NO_MATCH;
                Object primaryKeyValue = ((BeaconSavedRealmProxyInterface) object).realmGet$hashcode();
                if (primaryKeyValue != null) {
                    rowIndex = Table.nativeFindFirstInt(tableNativePtr, pkColumnIndex, ((BeaconSavedRealmProxyInterface) object).realmGet$hashcode());
                }
                if (rowIndex == TableOrView.NO_MATCH) {
                    rowIndex = table.addEmptyRowWithPrimaryKey(((BeaconSavedRealmProxyInterface) object).realmGet$hashcode(), false);
                } else {
                    Table.throwDuplicatePrimaryKeyException(primaryKeyValue);
                }
                cache.put(object, rowIndex);
                Table.nativeSetLong(tableNativePtr, columnInfo.beaconTypeIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$beaconType(), false);
                String realmGet$beaconAddress = ((BeaconSavedRealmProxyInterface)object).realmGet$beaconAddress();
                if (realmGet$beaconAddress != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.beaconAddressIndex, rowIndex, realmGet$beaconAddress, false);
                }
                String realmGet$UUID = ((BeaconSavedRealmProxyInterface)object).realmGet$UUID();
                if (realmGet$UUID != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.UUIDIndex, rowIndex, realmGet$UUID, false);
                }
                String realmGet$Major = ((BeaconSavedRealmProxyInterface)object).realmGet$Major();
                if (realmGet$Major != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.MajorIndex, rowIndex, realmGet$Major, false);
                }
                String realmGet$Minor = ((BeaconSavedRealmProxyInterface)object).realmGet$Minor();
                if (realmGet$Minor != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.MinorIndex, rowIndex, realmGet$Minor, false);
                }
                Table.nativeSetLong(tableNativePtr, columnInfo.txPowerIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$txPower(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.RSSIIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$RSSI(), false);
                Table.nativeSetDouble(tableNativePtr, columnInfo.distanceIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$distance(), false);
                java.util.Date realmGet$lastSeen = ((BeaconSavedRealmProxyInterface)object).realmGet$lastSeen();
                if (realmGet$lastSeen != null) {
                    Table.nativeSetTimestamp(tableNativePtr, columnInfo.lastSeenIndex, rowIndex, realmGet$lastSeen.getTime(), false);
                }
                Table.nativeSetLong(tableNativePtr, columnInfo.lastMinuteSeenIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$lastMinuteSeen(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.manufacturerIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$manufacturer(), false);
                String realmGet$URL = ((BeaconSavedRealmProxyInterface)object).realmGet$URL();
                if (realmGet$URL != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.URLIndex, rowIndex, realmGet$URL, false);
                }
                String realmGet$namespaceId = ((BeaconSavedRealmProxyInterface)object).realmGet$namespaceId();
                if (realmGet$namespaceId != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.namespaceIdIndex, rowIndex, realmGet$namespaceId, false);
                }
                String realmGet$instanceId = ((BeaconSavedRealmProxyInterface)object).realmGet$instanceId();
                if (realmGet$instanceId != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.instanceIdIndex, rowIndex, realmGet$instanceId, false);
                }
                Table.nativeSetBoolean(tableNativePtr, columnInfo.hasTelemetryDataIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$hasTelemetryData(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.telemetryVersionIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$telemetryVersion(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.batteryMilliVoltsIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$batteryMilliVolts(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.temperatureIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$temperature(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.pduCountIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$pduCount(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.uptimeIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$uptime(), false);
            }
        }
    }

    public static long insertOrUpdate(Realm realm, com.bridou_n.beaconscanner.models.BeaconSaved object, Map<RealmModel,Long> cache) {
        if (object instanceof RealmObjectProxy && ((RealmObjectProxy)object).realmGet$proxyState().getRealm$realm() != null && ((RealmObjectProxy)object).realmGet$proxyState().getRealm$realm().getPath().equals(realm.getPath())) {
            return ((RealmObjectProxy)object).realmGet$proxyState().getRow$realm().getIndex();
        }
        Table table = realm.getTable(com.bridou_n.beaconscanner.models.BeaconSaved.class);
        long tableNativePtr = table.getNativeTablePointer();
        BeaconSavedColumnInfo columnInfo = (BeaconSavedColumnInfo) realm.schema.getColumnInfo(com.bridou_n.beaconscanner.models.BeaconSaved.class);
        long pkColumnIndex = table.getPrimaryKey();
        long rowIndex = TableOrView.NO_MATCH;
        Object primaryKeyValue = ((BeaconSavedRealmProxyInterface) object).realmGet$hashcode();
        if (primaryKeyValue != null) {
            rowIndex = Table.nativeFindFirstInt(tableNativePtr, pkColumnIndex, ((BeaconSavedRealmProxyInterface) object).realmGet$hashcode());
        }
        if (rowIndex == TableOrView.NO_MATCH) {
            rowIndex = table.addEmptyRowWithPrimaryKey(((BeaconSavedRealmProxyInterface) object).realmGet$hashcode(), false);
        }
        cache.put(object, rowIndex);
        Table.nativeSetLong(tableNativePtr, columnInfo.beaconTypeIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$beaconType(), false);
        String realmGet$beaconAddress = ((BeaconSavedRealmProxyInterface)object).realmGet$beaconAddress();
        if (realmGet$beaconAddress != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.beaconAddressIndex, rowIndex, realmGet$beaconAddress, false);
        } else {
            Table.nativeSetNull(tableNativePtr, columnInfo.beaconAddressIndex, rowIndex, false);
        }
        String realmGet$UUID = ((BeaconSavedRealmProxyInterface)object).realmGet$UUID();
        if (realmGet$UUID != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.UUIDIndex, rowIndex, realmGet$UUID, false);
        } else {
            Table.nativeSetNull(tableNativePtr, columnInfo.UUIDIndex, rowIndex, false);
        }
        String realmGet$Major = ((BeaconSavedRealmProxyInterface)object).realmGet$Major();
        if (realmGet$Major != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.MajorIndex, rowIndex, realmGet$Major, false);
        } else {
            Table.nativeSetNull(tableNativePtr, columnInfo.MajorIndex, rowIndex, false);
        }
        String realmGet$Minor = ((BeaconSavedRealmProxyInterface)object).realmGet$Minor();
        if (realmGet$Minor != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.MinorIndex, rowIndex, realmGet$Minor, false);
        } else {
            Table.nativeSetNull(tableNativePtr, columnInfo.MinorIndex, rowIndex, false);
        }
        Table.nativeSetLong(tableNativePtr, columnInfo.txPowerIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$txPower(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.RSSIIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$RSSI(), false);
        Table.nativeSetDouble(tableNativePtr, columnInfo.distanceIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$distance(), false);
        java.util.Date realmGet$lastSeen = ((BeaconSavedRealmProxyInterface)object).realmGet$lastSeen();
        if (realmGet$lastSeen != null) {
            Table.nativeSetTimestamp(tableNativePtr, columnInfo.lastSeenIndex, rowIndex, realmGet$lastSeen.getTime(), false);
        } else {
            Table.nativeSetNull(tableNativePtr, columnInfo.lastSeenIndex, rowIndex, false);
        }
        Table.nativeSetLong(tableNativePtr, columnInfo.lastMinuteSeenIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$lastMinuteSeen(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.manufacturerIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$manufacturer(), false);
        String realmGet$URL = ((BeaconSavedRealmProxyInterface)object).realmGet$URL();
        if (realmGet$URL != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.URLIndex, rowIndex, realmGet$URL, false);
        } else {
            Table.nativeSetNull(tableNativePtr, columnInfo.URLIndex, rowIndex, false);
        }
        String realmGet$namespaceId = ((BeaconSavedRealmProxyInterface)object).realmGet$namespaceId();
        if (realmGet$namespaceId != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.namespaceIdIndex, rowIndex, realmGet$namespaceId, false);
        } else {
            Table.nativeSetNull(tableNativePtr, columnInfo.namespaceIdIndex, rowIndex, false);
        }
        String realmGet$instanceId = ((BeaconSavedRealmProxyInterface)object).realmGet$instanceId();
        if (realmGet$instanceId != null) {
            Table.nativeSetString(tableNativePtr, columnInfo.instanceIdIndex, rowIndex, realmGet$instanceId, false);
        } else {
            Table.nativeSetNull(tableNativePtr, columnInfo.instanceIdIndex, rowIndex, false);
        }
        Table.nativeSetBoolean(tableNativePtr, columnInfo.hasTelemetryDataIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$hasTelemetryData(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.telemetryVersionIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$telemetryVersion(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.batteryMilliVoltsIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$batteryMilliVolts(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.temperatureIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$temperature(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.pduCountIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$pduCount(), false);
        Table.nativeSetLong(tableNativePtr, columnInfo.uptimeIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$uptime(), false);
        return rowIndex;
    }

    public static void insertOrUpdate(Realm realm, Iterator<? extends RealmModel> objects, Map<RealmModel,Long> cache) {
        Table table = realm.getTable(com.bridou_n.beaconscanner.models.BeaconSaved.class);
        long tableNativePtr = table.getNativeTablePointer();
        BeaconSavedColumnInfo columnInfo = (BeaconSavedColumnInfo) realm.schema.getColumnInfo(com.bridou_n.beaconscanner.models.BeaconSaved.class);
        long pkColumnIndex = table.getPrimaryKey();
        com.bridou_n.beaconscanner.models.BeaconSaved object = null;
        while (objects.hasNext()) {
            object = (com.bridou_n.beaconscanner.models.BeaconSaved) objects.next();
            if(!cache.containsKey(object)) {
                if (object instanceof RealmObjectProxy && ((RealmObjectProxy)object).realmGet$proxyState().getRealm$realm() != null && ((RealmObjectProxy)object).realmGet$proxyState().getRealm$realm().getPath().equals(realm.getPath())) {
                    cache.put(object, ((RealmObjectProxy)object).realmGet$proxyState().getRow$realm().getIndex());
                    continue;
                }
                long rowIndex = TableOrView.NO_MATCH;
                Object primaryKeyValue = ((BeaconSavedRealmProxyInterface) object).realmGet$hashcode();
                if (primaryKeyValue != null) {
                    rowIndex = Table.nativeFindFirstInt(tableNativePtr, pkColumnIndex, ((BeaconSavedRealmProxyInterface) object).realmGet$hashcode());
                }
                if (rowIndex == TableOrView.NO_MATCH) {
                    rowIndex = table.addEmptyRowWithPrimaryKey(((BeaconSavedRealmProxyInterface) object).realmGet$hashcode(), false);
                }
                cache.put(object, rowIndex);
                Table.nativeSetLong(tableNativePtr, columnInfo.beaconTypeIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$beaconType(), false);
                String realmGet$beaconAddress = ((BeaconSavedRealmProxyInterface)object).realmGet$beaconAddress();
                if (realmGet$beaconAddress != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.beaconAddressIndex, rowIndex, realmGet$beaconAddress, false);
                } else {
                    Table.nativeSetNull(tableNativePtr, columnInfo.beaconAddressIndex, rowIndex, false);
                }
                String realmGet$UUID = ((BeaconSavedRealmProxyInterface)object).realmGet$UUID();
                if (realmGet$UUID != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.UUIDIndex, rowIndex, realmGet$UUID, false);
                } else {
                    Table.nativeSetNull(tableNativePtr, columnInfo.UUIDIndex, rowIndex, false);
                }
                String realmGet$Major = ((BeaconSavedRealmProxyInterface)object).realmGet$Major();
                if (realmGet$Major != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.MajorIndex, rowIndex, realmGet$Major, false);
                } else {
                    Table.nativeSetNull(tableNativePtr, columnInfo.MajorIndex, rowIndex, false);
                }
                String realmGet$Minor = ((BeaconSavedRealmProxyInterface)object).realmGet$Minor();
                if (realmGet$Minor != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.MinorIndex, rowIndex, realmGet$Minor, false);
                } else {
                    Table.nativeSetNull(tableNativePtr, columnInfo.MinorIndex, rowIndex, false);
                }
                Table.nativeSetLong(tableNativePtr, columnInfo.txPowerIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$txPower(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.RSSIIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$RSSI(), false);
                Table.nativeSetDouble(tableNativePtr, columnInfo.distanceIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$distance(), false);
                java.util.Date realmGet$lastSeen = ((BeaconSavedRealmProxyInterface)object).realmGet$lastSeen();
                if (realmGet$lastSeen != null) {
                    Table.nativeSetTimestamp(tableNativePtr, columnInfo.lastSeenIndex, rowIndex, realmGet$lastSeen.getTime(), false);
                } else {
                    Table.nativeSetNull(tableNativePtr, columnInfo.lastSeenIndex, rowIndex, false);
                }
                Table.nativeSetLong(tableNativePtr, columnInfo.lastMinuteSeenIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$lastMinuteSeen(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.manufacturerIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$manufacturer(), false);
                String realmGet$URL = ((BeaconSavedRealmProxyInterface)object).realmGet$URL();
                if (realmGet$URL != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.URLIndex, rowIndex, realmGet$URL, false);
                } else {
                    Table.nativeSetNull(tableNativePtr, columnInfo.URLIndex, rowIndex, false);
                }
                String realmGet$namespaceId = ((BeaconSavedRealmProxyInterface)object).realmGet$namespaceId();
                if (realmGet$namespaceId != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.namespaceIdIndex, rowIndex, realmGet$namespaceId, false);
                } else {
                    Table.nativeSetNull(tableNativePtr, columnInfo.namespaceIdIndex, rowIndex, false);
                }
                String realmGet$instanceId = ((BeaconSavedRealmProxyInterface)object).realmGet$instanceId();
                if (realmGet$instanceId != null) {
                    Table.nativeSetString(tableNativePtr, columnInfo.instanceIdIndex, rowIndex, realmGet$instanceId, false);
                } else {
                    Table.nativeSetNull(tableNativePtr, columnInfo.instanceIdIndex, rowIndex, false);
                }
                Table.nativeSetBoolean(tableNativePtr, columnInfo.hasTelemetryDataIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$hasTelemetryData(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.telemetryVersionIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$telemetryVersion(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.batteryMilliVoltsIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$batteryMilliVolts(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.temperatureIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$temperature(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.pduCountIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$pduCount(), false);
                Table.nativeSetLong(tableNativePtr, columnInfo.uptimeIndex, rowIndex, ((BeaconSavedRealmProxyInterface)object).realmGet$uptime(), false);
            }
        }
    }

    public static com.bridou_n.beaconscanner.models.BeaconSaved createDetachedCopy(com.bridou_n.beaconscanner.models.BeaconSaved realmObject, int currentDepth, int maxDepth, Map<RealmModel, CacheData<RealmModel>> cache) {
        if (currentDepth > maxDepth || realmObject == null) {
            return null;
        }
        CacheData<RealmModel> cachedObject = cache.get(realmObject);
        com.bridou_n.beaconscanner.models.BeaconSaved unmanagedObject;
        if (cachedObject != null) {
            // Reuse cached object or recreate it because it was encountered at a lower depth.
            if (currentDepth >= cachedObject.minDepth) {
                return (com.bridou_n.beaconscanner.models.BeaconSaved)cachedObject.object;
            } else {
                unmanagedObject = (com.bridou_n.beaconscanner.models.BeaconSaved)cachedObject.object;
                cachedObject.minDepth = currentDepth;
            }
        } else {
            unmanagedObject = new com.bridou_n.beaconscanner.models.BeaconSaved();
            cache.put(realmObject, new RealmObjectProxy.CacheData(currentDepth, unmanagedObject));
        }
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$hashcode(((BeaconSavedRealmProxyInterface) realmObject).realmGet$hashcode());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$beaconType(((BeaconSavedRealmProxyInterface) realmObject).realmGet$beaconType());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$beaconAddress(((BeaconSavedRealmProxyInterface) realmObject).realmGet$beaconAddress());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$UUID(((BeaconSavedRealmProxyInterface) realmObject).realmGet$UUID());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$Major(((BeaconSavedRealmProxyInterface) realmObject).realmGet$Major());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$Minor(((BeaconSavedRealmProxyInterface) realmObject).realmGet$Minor());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$txPower(((BeaconSavedRealmProxyInterface) realmObject).realmGet$txPower());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$RSSI(((BeaconSavedRealmProxyInterface) realmObject).realmGet$RSSI());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$distance(((BeaconSavedRealmProxyInterface) realmObject).realmGet$distance());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$lastSeen(((BeaconSavedRealmProxyInterface) realmObject).realmGet$lastSeen());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$lastMinuteSeen(((BeaconSavedRealmProxyInterface) realmObject).realmGet$lastMinuteSeen());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$manufacturer(((BeaconSavedRealmProxyInterface) realmObject).realmGet$manufacturer());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$URL(((BeaconSavedRealmProxyInterface) realmObject).realmGet$URL());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$namespaceId(((BeaconSavedRealmProxyInterface) realmObject).realmGet$namespaceId());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$instanceId(((BeaconSavedRealmProxyInterface) realmObject).realmGet$instanceId());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$hasTelemetryData(((BeaconSavedRealmProxyInterface) realmObject).realmGet$hasTelemetryData());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$telemetryVersion(((BeaconSavedRealmProxyInterface) realmObject).realmGet$telemetryVersion());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$batteryMilliVolts(((BeaconSavedRealmProxyInterface) realmObject).realmGet$batteryMilliVolts());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$temperature(((BeaconSavedRealmProxyInterface) realmObject).realmGet$temperature());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$pduCount(((BeaconSavedRealmProxyInterface) realmObject).realmGet$pduCount());
        ((BeaconSavedRealmProxyInterface) unmanagedObject).realmSet$uptime(((BeaconSavedRealmProxyInterface) realmObject).realmGet$uptime());
        return unmanagedObject;
    }

    static com.bridou_n.beaconscanner.models.BeaconSaved update(Realm realm, com.bridou_n.beaconscanner.models.BeaconSaved realmObject, com.bridou_n.beaconscanner.models.BeaconSaved newObject, Map<RealmModel, RealmObjectProxy> cache) {
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$beaconType(((BeaconSavedRealmProxyInterface) newObject).realmGet$beaconType());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$beaconAddress(((BeaconSavedRealmProxyInterface) newObject).realmGet$beaconAddress());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$UUID(((BeaconSavedRealmProxyInterface) newObject).realmGet$UUID());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$Major(((BeaconSavedRealmProxyInterface) newObject).realmGet$Major());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$Minor(((BeaconSavedRealmProxyInterface) newObject).realmGet$Minor());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$txPower(((BeaconSavedRealmProxyInterface) newObject).realmGet$txPower());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$RSSI(((BeaconSavedRealmProxyInterface) newObject).realmGet$RSSI());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$distance(((BeaconSavedRealmProxyInterface) newObject).realmGet$distance());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$lastSeen(((BeaconSavedRealmProxyInterface) newObject).realmGet$lastSeen());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$lastMinuteSeen(((BeaconSavedRealmProxyInterface) newObject).realmGet$lastMinuteSeen());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$manufacturer(((BeaconSavedRealmProxyInterface) newObject).realmGet$manufacturer());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$URL(((BeaconSavedRealmProxyInterface) newObject).realmGet$URL());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$namespaceId(((BeaconSavedRealmProxyInterface) newObject).realmGet$namespaceId());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$instanceId(((BeaconSavedRealmProxyInterface) newObject).realmGet$instanceId());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$hasTelemetryData(((BeaconSavedRealmProxyInterface) newObject).realmGet$hasTelemetryData());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$telemetryVersion(((BeaconSavedRealmProxyInterface) newObject).realmGet$telemetryVersion());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$batteryMilliVolts(((BeaconSavedRealmProxyInterface) newObject).realmGet$batteryMilliVolts());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$temperature(((BeaconSavedRealmProxyInterface) newObject).realmGet$temperature());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$pduCount(((BeaconSavedRealmProxyInterface) newObject).realmGet$pduCount());
        ((BeaconSavedRealmProxyInterface) realmObject).realmSet$uptime(((BeaconSavedRealmProxyInterface) newObject).realmGet$uptime());
        return realmObject;
    }

    @Override
    public String toString() {
        if (!RealmObject.isValid(this)) {
            return "Invalid object";
        }
        StringBuilder stringBuilder = new StringBuilder("BeaconSaved = [");
        stringBuilder.append("{hashcode:");
        stringBuilder.append(realmGet$hashcode());
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{beaconType:");
        stringBuilder.append(realmGet$beaconType());
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{beaconAddress:");
        stringBuilder.append(realmGet$beaconAddress() != null ? realmGet$beaconAddress() : "null");
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{UUID:");
        stringBuilder.append(realmGet$UUID() != null ? realmGet$UUID() : "null");
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{Major:");
        stringBuilder.append(realmGet$Major() != null ? realmGet$Major() : "null");
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{Minor:");
        stringBuilder.append(realmGet$Minor() != null ? realmGet$Minor() : "null");
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{txPower:");
        stringBuilder.append(realmGet$txPower());
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{RSSI:");
        stringBuilder.append(realmGet$RSSI());
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{distance:");
        stringBuilder.append(realmGet$distance());
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{lastSeen:");
        stringBuilder.append(realmGet$lastSeen() != null ? realmGet$lastSeen() : "null");
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{lastMinuteSeen:");
        stringBuilder.append(realmGet$lastMinuteSeen());
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{manufacturer:");
        stringBuilder.append(realmGet$manufacturer());
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{URL:");
        stringBuilder.append(realmGet$URL() != null ? realmGet$URL() : "null");
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{namespaceId:");
        stringBuilder.append(realmGet$namespaceId() != null ? realmGet$namespaceId() : "null");
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{instanceId:");
        stringBuilder.append(realmGet$instanceId() != null ? realmGet$instanceId() : "null");
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{hasTelemetryData:");
        stringBuilder.append(realmGet$hasTelemetryData());
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{telemetryVersion:");
        stringBuilder.append(realmGet$telemetryVersion());
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{batteryMilliVolts:");
        stringBuilder.append(realmGet$batteryMilliVolts());
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{temperature:");
        stringBuilder.append(realmGet$temperature());
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{pduCount:");
        stringBuilder.append(realmGet$pduCount());
        stringBuilder.append("}");
        stringBuilder.append(",");
        stringBuilder.append("{uptime:");
        stringBuilder.append(realmGet$uptime());
        stringBuilder.append("}");
        stringBuilder.append("]");
        return stringBuilder.toString();
    }

    @Override
    public ProxyState realmGet$proxyState() {
        return proxyState;
    }

    @Override
    public int hashCode() {
        String realmName = proxyState.getRealm$realm().getPath();
        String tableName = proxyState.getRow$realm().getTable().getName();
        long rowIndex = proxyState.getRow$realm().getIndex();

        int result = 17;
        result = 31 * result + ((realmName != null) ? realmName.hashCode() : 0);
        result = 31 * result + ((tableName != null) ? tableName.hashCode() : 0);
        result = 31 * result + (int) (rowIndex ^ (rowIndex >>> 32));
        return result;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        BeaconSavedRealmProxy aBeaconSaved = (BeaconSavedRealmProxy)o;

        String path = proxyState.getRealm$realm().getPath();
        String otherPath = aBeaconSaved.proxyState.getRealm$realm().getPath();
        if (path != null ? !path.equals(otherPath) : otherPath != null) return false;

        String tableName = proxyState.getRow$realm().getTable().getName();
        String otherTableName = aBeaconSaved.proxyState.getRow$realm().getTable().getName();
        if (tableName != null ? !tableName.equals(otherTableName) : otherTableName != null) return false;

        if (proxyState.getRow$realm().getIndex() != aBeaconSaved.proxyState.getRow$realm().getIndex()) return false;

        return true;
    }

}
