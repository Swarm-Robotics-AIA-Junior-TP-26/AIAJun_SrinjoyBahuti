

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

CSV_PATH = "ec704193-c88d-4f4b-8e18-e2cb387c3c47.csv"  # change if needed


df = pd.read_csv(CSV_PATH)


df["timestamp"] = pd.to_datetime(df["timestamp"], utc=True, errors="coerce")
df["altitude_m"] = pd.to_numeric(df["altitude_m"], errors="coerce")
df["latitude"]   = pd.to_numeric(df["latitude"], errors="coerce")
df["longitude"]  = pd.to_numeric(df["longitude"], errors="coerce")


alt = df["altitude_m"].copy()
alt[(alt < -50) | (alt > 10000)] = np.nan


order = df["timestamp"].argsort()
alt_sorted = alt.iloc[order]
spikes = alt_sorted.diff().abs() > 100.0  # threshold in meters per sample
alt.iloc[alt_sorted.index[spikes.fillna(False)]] = np.nan

df["altitude_clean"] = alt.ffill().bfill()
df["latitude"]  = df["latitude"].ffill().bfill()
df["longitude"] = df["longitude"].ffill().bfill()


print("=== BASIC ALTITUDE STATS (cleaned) ===")
print("Mean:", df["altitude_clean"].mean())
print("Min :", df["altitude_clean"].min())
print("Max :", df["altitude_clean"].max())


plt.figure(figsize=(10,4))
plt.plot(df["timestamp"], df["altitude_clean"])
plt.xlabel("Timestamp (UTC)")
plt.ylabel("Altitude (m)")
plt.title("Altitude vs Time (cleaned)")
plt.tight_layout()
plt.savefig("altitude_vs_time.png", dpi=160)
plt.close()
print("Saved: altitude_vs_time.png")


if "flight_mode" in df.columns:
    for mode, g in df.groupby(df["flight_mode"].astype(str).fillna("Unknown")):
        plt.figure(figsize=(6,6))
        plt.scatter(g["longitude"], g["latitude"], s=8)
        plt.xlabel("Longitude (deg)")
        plt.ylabel("Latitude (deg)")
        plt.title(f"GPS Track — mode: {mode}")
        plt.axis("equal")
        out = f"gps_by_mode_{mode.replace(' ','_').replace('/','_')}.png"
        plt.tight_layout()
        plt.savefig(out, dpi=160)
        plt.close()
        print("Saved:", out)
else:
    plt.figure(figsize=(6,6))
    plt.scatter(df["longitude"], df["latitude"], s=8)
    plt.xlabel("Longitude (deg)")
    plt.ylabel("Latitude (deg)")
    plt.title("GPS Track")
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig("gps_track.png", dpi=160)
    plt.close()
    print("Saved: gps_track.png")

df[["timestamp","altitude_clean","latitude","longitude","flight_mode"]].to_csv(
    "cleaned_drone_data.csv", index=False
)
print("Saved: cleaned_drone_data.csv")
