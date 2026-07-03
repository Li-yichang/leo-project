# leo-project

本專案聚焦於低軌衛星（Low Earth Orbit, LEO）網路中的資料傳輸效率問題。透過 NS-3 模擬環境，研究在具備多條 Inter-Satellite Links（ISLs）的情境下，封包壓縮（Data Compression）與路由策略對端到端延遲（End-to-End Latency）與吞吐量（Throughput）的影響。

簡要說明如下：在頻寬受限且拓樸快速變動的 LEO 網路中，本研究嘗試在延遲與頻寬利用率之間取得更佳的平衡。

---

## Motivation

LEO 衛星網路具有低延遲與全球覆蓋的優勢，但同時也面臨以下挑戰：

- Inter-Satellite Link 頻寬有限且資源昂貴
- 網路拓樸隨衛星移動快速變化
- 路由決策對效能具有高度敏感性
- 延遲與吞吐量之間存在明顯 trade-off

本專案假設，若能在適當的節點與時機進行封包壓縮，並搭配合適的路由策略，則可在不顯著增加延遲的情況下，提升整體頻寬使用效率。


---

## System Overview

- Simulation Platform: NS-3
- Network Type: LEO Satellite Network with ISLs
- Application Layer:
  - Custom Compression Proxy Application
  - Support for multiple compression ratios

---

## Experimental Design

控制變因包含：

- Compression ratio
- Packet size
- Number of transmitted packets
- Routing path selection

評估指標包含：

- End-to-end delay
- Throughput (uplink / downlink)
- Total transmission time

基礎實驗：
- 適度壓縮 (ratio=0.5)：在僅小幅降低下行吞吐量的情況下，將總傳輸時間縮短約 42-44%。 
- 過度壓縮 (ratio=0.2)：雖進一步減少時間，卻使吞吐量下降逾 30%。 
- 拓撲差異：不論傳輸時間或吞吐量，六顆衛星與七顆衛星的結果較為接近（差距僅在千分位），在圖表中線段近乎重疊；唯獨距離最遠的第五顆衛星差異較顯著。

互動式功能：除了系統自動選擇最適路徑，亦讓觀測者可手動輸入路徑，例如分析略過其中一顆衛星後的變化。
-系統對於微小的路徑長度變化展現出高度的魯棒性，不同路徑間的效能差異不明顯，在下面的圖表中可以看到除了距離最遠的第五顆衛星，結果都差異及小，近乎到千分位，也就是說比起複雜的路由最佳化，專注於提升單個node的處理效率（例如資料壓縮）是更有效的效能提升策略。 
---

## Results and Observations

實驗結果顯示：
- 適度的資料壓縮可有效提升下行吞吐量
- 在多 ISL 拓樸下，路由選擇對整體效能的影響不明顯，系統魯棒性高，比起複雜的路由最佳化，專注於提升單個節點的處理效率（例如資料壓縮）是更有效的效能提升策略
- 過高的壓縮比例可能因處理延遲而抵銷頻寬收益
- 大封包效益放大：10000 Bytes 封包從壓縮中獲得最大的絕對時間減少 

詳細實驗數據與分析結果請參考 `results/` 目錄。
<img width="1238" height="1754" alt="image" src="https://github.com/user-attachments/assets/e979c42b-044e-40fc-babc-25e08966b766" />
