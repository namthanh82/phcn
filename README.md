# Cài qmake + dev tools (lần đầu)
sudo apt update
sudo apt install -y qtbase5-dev qt5-qmake sip-dev

# Rebuild PyQt5 (mất 10-15 phút trên Pi 5)
cd ~/phcn/giaodienphuchoi/scripts
source .venv/bin/activate
pip install --force-reinstall --no-cache-dir PyQt5==5.15.11
