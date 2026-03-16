#run this script on your pc.
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

df = pd.read_csv("/Users/admin/Downloads/aaa/pid_tests.csv")

runs = df["run"].unique()

plt.figure(figsize=(10,6))

for run in runs:

    data = df[df["run"] == run]

    # use milliseconds directly
    t = data["time_ms"]

    pos = data["position"]
    sp = data["setpoint"].iloc[0]

    # ---- damping ratio ----
    error = pos - sp
    peaks, _ = find_peaks(np.abs(error))

    if len(peaks) >= 2:
        x1 = abs(error.iloc[peaks[0]])
        x2 = abs(error.iloc[peaks[1]])

        delta = np.log(x1/x2)
        zeta = delta / np.sqrt(4*np.pi**2 + delta**2)
    else:
        zeta = np.nan

    plt.plot(t, pos, label=f"Run {run}   ζ={zeta:.2f}")

# setpoint line
plt.axhline(sp, linestyle="--", color="black", label="Setpoint")

plt.xlabel("Time (ms)")
plt.ylabel("Position (degrees)")
plt.title("PID Step Responses with Damping Ratio")
plt.grid()
plt.legend()

plt.show()
