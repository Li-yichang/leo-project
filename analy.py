
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import statsmodels.formula.api as smf
import os
import glob

# === 資料夾設定 ===
csv_folder = "./"   # CSV 所在資料夾
outdir = "figures"
os.makedirs(outdir, exist_ok=True)

# === 讀取所有 CSV，並加上 PathID & HopCount ===
all_files = glob.glob(os.path.join(csv_folder, "*.csv"))
dfs = []

for f in all_files:
    path_name = os.path.basename(f).replace(".csv","")
    df_tmp = pd.read_csv(f)
    df_tmp.columns = [c.strip() for c in df_tmp.columns]

    # 重命名欄位
    rename_map = {
        'Ratio': 'CR',
        'PacketSize': 'Pkt',
        'Up(Mbps)': 'Up',
        'Down(Mbps)': 'Down',
        'TotalTime(s)': 'Latency'
    }
    df_tmp = df_tmp.rename(columns=rename_map)

    # PathID & HopCount
    df_tmp['PathID'] = path_name
    df_tmp['HopCount'] = len(path_name.split())
    dfs.append(df_tmp)

# 合併
df = pd.concat(dfs, ignore_index=True)

# === 純粹分析方式 ===
# 1️ 每條路徑 Latency vs PacketSize（固定 CR）
for cr in sorted(df['CR'].unique()):
    plt.figure()
    for path in df['PathID'].unique():
        sub = df[(df['PathID']==path) & (df['CR']==cr)]
        plt.plot(sub['Pkt'], sub['Latency'], marker='o', label=path)
    plt.title(f'Latency vs PacketSize (CR={cr})')
    plt.xlabel("Packet Size (byte)")
    plt.ylabel("Latency (s)")
    plt.grid(True)
    plt.legend(title="Path")
    plt.tight_layout()
    plt.savefig(f"{outdir}/Latency_vs_Pkt_CR{cr}.png", dpi=300)
    plt.close()

# 2️ 每條路徑 Down vs CR（固定 Pkt）
for pkt in sorted(df['Pkt'].unique()):
    plt.figure()
    for path in df['PathID'].unique():
        sub = df[(df['PathID']==path) & (df['Pkt']==pkt)]
        plt.plot(sub['CR'], sub['Down'], marker='o', label=path)
    plt.title(f'Downlink Throughput vs CR (Pkt={pkt})')
    plt.xlabel("Compression Ratio")
    plt.ylabel("Down Throughput (Mbps)")
    plt.grid(True)
    plt.legend(title="Path")
    plt.tight_layout()
    plt.savefig(f"{outdir}/Down_vs_CR_Pkt{pkt}.png", dpi=300)
    plt.close()



# 4️ Regression Analysis
model_latency = smf.ols('Latency ~ HopCount + Pkt + CR', data=df).fit()
print("\n=== Regression: HopCount, PacketSize, CR -> Latency ===")
print(model_latency.summary())
