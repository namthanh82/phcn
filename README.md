sudo apt update && sudo apt -y upgrade
sudo apt install -y python3 python3-venv python3-pip git \
                    libqt5gui5 libqt5widgets5 libqt5charts5 \
                    fonts-noto-core


sudo tee /etc/udev/rules.d/99-odrive.rules <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0d32", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0d33", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER
# logout/login lại để nhận group dialout


cd ~
git clone https://github.com/namthanh82/phcn.git
cd phcn/giaodienphuchoi/scripts
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt



cd ~/phcn/giaodienphuchoi/scripts
./runapp.sh


cat > runapp.sh <<'EOF'
#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
if [ -x ".venv/bin/python" ]; then PY=".venv/bin/python"; else PY="python3"; fi
echo "[GUI] Using $($PY --version)"
exec "$PY" GUI.py "$@"
EOF
chmod +x runapp.sh

