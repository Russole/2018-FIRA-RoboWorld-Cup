# 執行步驟

1. 使用 OpenCV (cv2) 來調整電腦視覺的各種參數，正確定位機器人的位置
2. 定位球的場球門、中線等座標
3. 針對比賽項目，執行對應的程式

# 程式內容

1. 電腦視覺

- 影像處理/capture_position_2.py
- 影像處理/CaptureHsv.py

2. 挑戰賽

- challenge 1 & 3 vs 3 : challenge_1_and_3v3/challenge1And3V3
  - 採用多執行緒的方式，共 3 個 Thread，分別負責攻擊、防守及電腦視覺
- challenge 2 : challenge2/
  - 採用多執行緒的方式同上
