# Xóa venv cũ
cd ~/phcn/giaodienphuchoi/scripts
rm -rf .venv

# Cài PyQt5 từ apt (đã compiled sẵn)
sudo apt update
sudo apt install -y python3-pyqt5 python3-pyqt5.qtchart python3-pyqt5.qtserialport

# Tạo venv mới, KHÔNG cài PyQt5 vào venv
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install pyyaml numpy

# Nói Python tìm PyQt5 từ system
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH

# Chạy thử
python GUI.py
