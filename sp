# Cài Qt5 từ system package (đã compiled, không cần qmake)
sudo apt update
sudo apt install -y \
    pyqt5-dev \
    pyqt5-dev-tools \
    qt5-qmake \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    libqt5gui5 \
    libqt5widgets5 \
    libqt5core5a \
    libqt5charts5 \
    python3-pyqt5

# Tạo venv MỚI (không compile PyQt5 từ pip nữa)
cd ~/phcn/giaodienphuchoi/scripts
rm -rf .venv
python3 -m venv .venv
source .venv/bin/activate

# Chỉ cài những thứ KHÔNG có sẵn trên hệ thống
pip install --upgrade pip
pip install pyyaml numpy
# KHÔNG cần pip install PyQt5 — đã có sẵn hệ thống
