package io.realm;


public interface BeaconSavedRealmProxyInterface {
    public int realmGet$hashcode();
    public void realmSet$hashcode(int value);
    public int realmGet$beaconType();
    public void realmSet$beaconType(int value);
    public String realmGet$beaconAddress();
    public void realmSet$beaconAddress(String value);
    public String realmGet$UUID();
    public void realmSet$UUID(String value);
    public String realmGet$Major();
    public void realmSet$Major(String value);
    public String realmGet$Minor();
    public void realmSet$Minor(String value);
    public int realmGet$txPower();
    public void realmSet$txPower(int value);
    public int realmGet$RSSI();
    public void realmSet$RSSI(int value);
    public double realmGet$distance();
    public void realmSet$distance(double value);
    public java.util.Date realmGet$lastSeen();
    public void realmSet$lastSeen(java.util.Date value);
    public long realmGet$lastMinuteSeen();
    public void realmSet$lastMinuteSeen(long value);
    public int realmGet$manufacturer();
    public void realmSet$manufacturer(int value);
    public String realmGet$URL();
    public void realmSet$URL(String value);
    public String realmGet$namespaceId();
    public void realmSet$namespaceId(String value);
    public String realmGet$instanceId();
    public void realmSet$instanceId(String value);
    public boolean realmGet$hasTelemetryData();
    public void realmSet$hasTelemetryData(boolean value);
    public long realmGet$telemetryVersion();
    public void realmSet$telemetryVersion(long value);
    public long realmGet$batteryMilliVolts();
    public void realmSet$batteryMilliVolts(long value);
    public long realmGet$temperature();
    public void realmSet$temperature(long value);
    public long realmGet$pduCount();
    public void realmSet$pduCount(long value);
    public long realmGet$uptime();
    public void realmSet$uptime(long value);
}
