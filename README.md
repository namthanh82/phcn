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











Installing build dependencies ... done
  Getting requirements to build wheel ... done
  Preparing metadata (pyproject.toml) ... error
  error: subprocess-exited-with-error
  
  × Preparing metadata (pyproject.toml) did not run successfully.
  │ exit code: 1
  ╰─> [32 lines of output]
      pyproject.toml: line 14: the legacy use of 'license' is deprecated and will be removed in SIP v7.0.0, use an SPDX license expression and 'license-files' instead
      Traceback (most recent call last):
        File "/home/namthanh5555/phcn/giaodienphuchoi/scripts/.venv/lib/python3.13/site-packages/pip/_vendor/pyproject_hooks/_in_process/_in_process.py", line 389, in <module>
          main()
          ~~~~^^
        File "/home/namthanh5555/phcn/giaodienphuchoi/scripts/.venv/lib/python3.13/site-packages/pip/_vendor/pyproject_hooks/_in_process/_in_process.py", line 373, in main
          json_out["return_val"] = hook(**hook_input["kwargs"])
                                   ~~~~^^^^^^^^^^^^^^^^^^^^^^^^
        File "/home/namthanh5555/phcn/giaodienphuchoi/scripts/.venv/lib/python3.13/site-packages/pip/_vendor/pyproject_hooks/_in_process/_in_process.py", line 178, in prepare_metadata_for_build_wheel
          whl_basename = backend.build_wheel(metadata_directory, config_settings)
        File "/tmp/pip-build-env-_h4tioyr/overlay/lib/python3.13/site-packages/sipbuild/api.py", line 28, in build_wheel
          project = AbstractProject.bootstrap('wheel',
                  arguments=_convert_config_settings(config_settings))
        File "/tmp/pip-build-env-_h4tioyr/overlay/lib/python3.13/site-packages/sipbuild/abstract_project.py", line 74, in bootstrap
          project.setup(pyproject, tool, tool_description)
          ~~~~~~~~~~~~~^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        File "/tmp/pip-build-env-_h4tioyr/overlay/lib/python3.13/site-packages/sipbuild/project.py", line 661, in setup
          self.apply_user_defaults(tool)
          ~~~~~~~~~~~~~~~~~~~~~~~~^^^^^^
        File "/tmp/pip-install-no448cdd/pyqt5_984bbfbb2a53409fa55574bb0b5b0bdb/project.py", line 68, in apply_user_defaults
          super().apply_user_defaults(tool)
          ~~~~~~~~~~~~~~~~~~~~~~~~~~~^^^^^^
        File "/tmp/pip-build-env-_h4tioyr/overlay/lib/python3.13/site-packages/pyqtbuild/project.py", line 51, in apply_user_defaults
          super().apply_user_defaults(tool)
          ~~~~~~~~~~~~~~~~~~~~~~~~~~~^^^^^^
        File "/tmp/pip-build-env-_h4tioyr/overlay/lib/python3.13/site-packages/sipbuild/project.py", line 248, in apply_user_defaults
          self.builder.apply_user_defaults(tool)
          ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^^^^^^
        File "/tmp/pip-build-env-_h4tioyr/overlay/lib/python3.13/site-packages/pyqtbuild/builder.py", line 49, in apply_user_defaults
          raise PyProjectOptionException('qmake',
                  "specify a working qmake or add it to PATH")
      sipbuild.pyproject.PyProjectOptionException
      [end of output]
  
  note: This error originates from a subprocess, and is likely not a problem with pip.
error: metadata-generation-failed

× Encountered error while generating package metadata.
╰─> PyQt5

note: This is an issue with the package mentioned above, not pip.
hint: See above for details.


