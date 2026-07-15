  Using cached PyQt5-5.15.11.tar.gz (3.2 MB)
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
        File "/tmp/pip-build-env-gelr0l_k/overlay/lib/python3.13/site-packages/sipbuild/api.py", line 28, in build_wheel
          project = AbstractProject.bootstrap('wheel',
                  arguments=_convert_config_settings(config_settings))
        File "/tmp/pip-build-env-gelr0l_k/overlay/lib/python3.13/site-packages/sipbuild/abstract_project.py", line 74, in bootstrap
          project.setup(pyproject, tool, tool_description)
          ~~~~~~~~~~~~~^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        File "/tmp/pip-build-env-gelr0l_k/overlay/lib/python3.13/site-packages/sipbuild/project.py", line 661, in setup
          self.apply_user_defaults(tool)
          ~~~~~~~~~~~~~~~~~~~~~~~~^^^^^^
        File "/tmp/pip-install-hu987rh5/pyqt5_8342aea1227b41e0a834e6c5b6cead76/project.py", line 68, in apply_user_defaults
          super().apply_user_defaults(tool)
          ~~~~~~~~~~~~~~~~~~~~~~~~~~~^^^^^^
        File "/tmp/pip-build-env-gelr0l_k/overlay/lib/python3.13/site-packages/pyqtbuild/project.py", line 51, in apply_user_defaults
          super().apply_user_defaults(tool)
          ~~~~~~~~~~~~~~~~~~~~~~~~~~~^^^^^^
        File "/tmp/pip-build-env-gelr0l_k/overlay/lib/python3.13/site-packages/sipbuild/project.py", line 248, in apply_user_defaults
          self.builder.apply_user_defaults(tool)
          ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^^^^^^
        File "/tmp/pip-build-env-gelr0l_k/overlay/lib/python3.13/site-packages/pyqtbuild/builder.py", line 49, in apply_user_defaults
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
