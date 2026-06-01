import pandas as pd
import numpy as np

# --------------------------------------------------
# User settings
# --------------------------------------------------
csv_file = "logfile122624-AllP0-100-S1-9BIGreps30.csv"
n_bootstrap =  10000
n_samples = 180

# --------------------------------------------------
# Read CSV file
# --------------------------------------------------
df = pd.read_csv(csv_file, header=None)

# Extract 9th column (index 8)
col = df.iloc[:, 8].to_numpy()

# Verify sufficient data
if len(col) < 360:
    raise ValueError(
        f"Expected at least 360 rows in column 9, found {len(col)}"
    )

# First 180 values = Va
Va = col[1:181].astype(float)
#print("First va",Va[0],Va[-1],len(Va))

# Second 180 values = Vb
Vb = col[181:361].astype(float)

#print("First Vb",Vb[0],Vb[-1],len(Vb))

# --------------------------------------------------
# Bootstrap
# --------------------------------------------------
ratios = np.empty(n_bootstrap)

for i in range(n_bootstrap):

    # Resample with replacement
    Va_boot = np.random.choice(Va, size=n_samples, replace=True)
    Vb_boot = np.random.choice(Vb, size=n_samples, replace=True)

    # Compute bootstrap means

    Va_sum = np.sum(Va_boot)#.astype(float))
    Vb_sum = np.sum(Vb_boot)#.astype(float))

    #print("sums ",Va_sum,Vb_sum)

    # Volume ratio
    ratios[i] = (Va_sum-Vb_sum) / Vb_sum

# --------------------------------------------------
# Statistics
# --------------------------------------------------
ratio_mean = np.mean(ratios)
ratio_std = np.std(ratios, ddof=1)

ci_lower = np.percentile(ratios, 2.5)
ci_upper = np.percentile(ratios, 97.5)

cil = ratio_mean - 1.96*ratio_std/np.sqrt(180)
ciu = ratio_mean + 1.96*ratio_std/np.sqrt(180)
# --------------------------------------------------
# Results
# --------------------------------------------------
print(f"Bootstrap samples      : {n_bootstrap}")
print(f"Mean volume ratio      : {ratio_mean:.6f}")
print(f"Std. dev. of ratios    : {ratio_std:.6f}")
print(f"95% Confidence Interval: [{cil:.6f}, {ciu:.6f}]")