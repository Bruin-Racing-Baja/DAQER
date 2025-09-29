import pandas as pd
import matplotlib.pyplot as plt
import os

directory = "current_plots"

for filename in os.listdir(directory):
    if filename.endswith(".csv"):
        filepath = os.path.join(directory, filename)
        
        try:
            df = pd.read_csv(filepath)
            df.columns = df.columns.str.strip()  # clean headers

            print("Columns:", df.columns.tolist())
            print(df.head())

            # Relative time in seconds
            x = (df["Timestamp (us)"] - df["Timestamp (us)"].iloc[0]) / 1e6

            # Convert ADC to psi
            y = ((df["BrakeADC"] * 3.3 / 1023) - 0.5) * (2900 / 4)

            avg_dt = x.diff().mean()
            print(f"{filename}: avg dt = {avg_dt:.6f} s, rows = {len(df)}")

            plt.figure(figsize=(10, 6))
            plt.plot(x, y, marker=".", linestyle="-")
            plt.xlabel("Time (seconds)")
            plt.ylabel("Brake Pressure (psi)")
            plt.title(f"Brake Pressure vs Time\n{filename}")
            plt.grid(True)
            plt.tight_layout()
            plt.show()

        except Exception as e:
            print(f"Skipping {filename}: {e}")
