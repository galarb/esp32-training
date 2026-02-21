import csv
import math
import matplotlib.pyplot as plt

filename = "/Users/admin/Downloads/aaa/pid_tests.csv"

runs = {}

# =========================
# LOAD CSV
# =========================

with open(filename, newline="") as csvfile:

    reader = csv.DictReader(csvfile)

    for row in reader:

        run = int(row["run"])
        time = float(row["time_ms"]) / 1000.0   # convert ms → seconds
        error = float(row["error"])
        position = float(row["position"])
        setpoint = float(row["setpoint"])
        Kp = float(row["Kp"])
        Ki = float(row["Ki"])
        Kd = float(row["Kd"])

        if run not in runs:

            runs[run] = {
                "time": [],
                "error": [],
                "position": [],
                "setpoint": setpoint,
                "Kp": Kp,
                "Ki": Ki,
                "Kd": Kd
            }

        runs[run]["time"].append(time)
        runs[run]["error"].append(error)
        runs[run]["position"].append(position)


# =========================
# ANALYSIS FUNCTIONS
# =========================

def calculate_zeta(setpoint, position):

    peak = max(position)

    overshoot = (peak - setpoint) / setpoint

    if overshoot <= 0:
        return None, overshoot

    zeta = -math.log(overshoot) / math.sqrt(math.pi**2 + math.log(overshoot)**2)

    return zeta, overshoot


def settling_time(time, position, setpoint, tolerance=0.02):

    band = setpoint * tolerance

    for i in range(len(position)):

        if abs(position[i] - setpoint) < band:

            stable = True

            for j in range(i, len(position)):
                if abs(position[j] - setpoint) > band:
                    stable = False
                    break

            if stable:
                return time[i]

    return None


def rise_time(time, position, setpoint):

    t10 = None
    t90 = None

    for i in range(len(position)):

        if t10 is None and position[i] >= 0.1 * setpoint:
            t10 = time[i]

        if t90 is None and position[i] >= 0.9 * setpoint:
            t90 = time[i]
            break

    if t10 is not None and t90 is not None:
        return t90 - t10

    return None


# =========================
# PLOT + ANALYZE
# =========================

plt.figure(figsize=(12,8))

results = []

for run in sorted(runs.keys()):

    data = runs[run]

    time = data["time"]
    error = data["error"]
    position = data["position"]
    setpoint = data["setpoint"]

    Kp = data["Kp"]
    Ki = data["Ki"]
    Kd = data["Kd"]

    zeta, overshoot = calculate_zeta(setpoint, position)

    st = settling_time(time, position, setpoint)

    rt = rise_time(time, position, setpoint)

    results.append({
        "run": run,
        "Kp": Kp,
        "Ki": Ki,
        "Kd": Kd,
        "zeta": zeta,
        "overshoot": overshoot,
        "settling": st,
        "rise": rt
    })

    label = f"Run {run}  Kp={Kp} Ki={Ki} Kd={Kd}"

    if zeta is not None:
        label += f"  ζ={zeta:.2f}"

    plt.plot(time, error, label=label)


# =========================
# GRAPH SETTINGS
# =========================

plt.axhline(0)
plt.xlabel("Time (seconds)")
plt.ylabel("Error (degrees)")
plt.title("PID Step Response Comparison")
plt.legend()
plt.grid()

plt.show()


# =========================
# PRINT RESULTS TABLE
# =========================

print("\nRESULTS:\n")

print("Run |  Kp   Ki   Kd  |  ζ    | Overshoot | Settling | Rise")
print("-----------------------------------------------------------")

for r in results:

    print(
        f"{r['run']:3} | "
        f"{r['Kp']:4.1f} {r['Ki']:4.1f} {r['Kd']:4.1f} | "
        f"{r['zeta'] if r['zeta'] else 0:5.2f} | "
        f"{r['overshoot']:9.3f} | "
        f"{r['settling'] if r['settling'] else 0:7.3f} | "
        f"{r['rise'] if r['rise'] else 0:7.3f}"
    )


# =========================
# FIND BEST PID
# =========================

best = None

for r in results:

    if r["zeta"] is None:
        continue

    if best is None or abs(r["zeta"] - 0.707) < abs(best["zeta"] - 0.707):
        best = r

print("\nBEST PID (closest to ζ=0.707):\n")

print(best)

