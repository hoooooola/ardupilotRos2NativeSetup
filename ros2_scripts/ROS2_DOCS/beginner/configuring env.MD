# ROS 2 環境變數設定

## 🔍 檢查環境變數
使用 `grep -i` 忽略大小寫，過濾出所有與 ROS 相關的環境變數。
```bash
printenv | grep -i ROS
```

## 🌐 設定通訊群組 (ROS_DOMAIN_ID)
`ROS_DOMAIN_ID` 用於區分同一個網路中的不同通訊群組（整數範圍：0~232）。

### 🛠️ 修正 `.bashrc` 設定
如果你之前的設定包含角括號（例如 `<1>`），請使用 `sed` 修正為正確的純數字格式：
```bash
# 修正 .bashrc 中的格式錯誤
sed -i 's/ROS_DOMAIN_ID=1/ROS_DOMAIN_ID=15/g' ~/.bashrc
```

### 🚀 載入與驗證
修改完設定檔後，需手動重新載入並確認生效狀況：
```bash
# 讓設定立刻在本視窗生效
source ~/.bashrc

# 確認目前的 Domain ID
echo $ROS_DOMAIN_ID
```

## 🔑 遠端存取與通訊 (SSH)
SSH (Secure Shell) 讓你可以透過網路安全地遠端操作其他電腦（例如機器人主機）。

### 📥 安裝與啟動
在 Ubuntu 中建議安裝 `openssh-server`：
```bash
# 更新軟體源並安裝 SSH 伺服器
sudo apt update
sudo apt install openssh-server -y

# 啟動並檢查服務狀態
sudo systemctl enable --now ssh
sudo systemctl status ssh
```

### 🖥️ 遠端登入範例
在您的筆電（或其他電腦）輸入以下指令來連線到此主機：
```bash
# 語法: ssh <用戶名>@<IP位址>
ssh user@10.30.2.68
```

## 🔒 本機通訊限制 (ROS_LOCALHOST_ONLY)

當您只想在目前這台電腦內進行 ROS 2 通訊，而不希望影響到網路上其他設備時（如教室場景），此變數非常有用。這會讓您的 Topic、Service 對外部網路不可見。

### 🛠️ 設定方法
```bash
# 暫時設定 (僅限此視窗)
export ROS_LOCALHOST_ONLY=1

# 永久設定 (寫入 .bashrc)
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
source ~/.bashrc
```

### 🔍 驗證狀態
```bash
echo $ROS_LOCALHOST_ONLY
```


