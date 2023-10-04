# Modularis - Documentation

[![Documentation Status](https://readthedocs.org/projects/modularis/badge/?version=latest)](https://modularis.readthedocs.io/en/latest/?badge=latest)


## Documentation
The [documentation website](https://modularis.readthedocs.io/en/latest/) can be accessed from the link.


## Dependencies
Below are the installation commands used to install the packages required to build the documentation on your local machine.
```Bash
sudo apt install python3-sphinx
```

Install pip3
```Bash
sudo apt install python3-pip
```

Use pip3 to install the Read the Docs template
```Bash
pip3 install sphinx-rtd-theme
```

Use pip3 to install recommommark to be able to use markdown files
```Bash
pip3 install recommonmark
```

Use pip3 to install sphinx_copybutton. It provides a copy button of code blocks.
```Bash
pip3 install sphinx_copybutton
```

Use pip3 to install sphinx_design. It provides additional formatting options.
```Bash
pip3 install sphinx_design
```


## File Types
The documentation can be written using either markdown (.md) or restructured text (.rst) formats.

### Notes
It is important that any new files created for the documentation are included in a toctree specified in a .rst file. You can use the existing documentation as an example.


## Build
You can build the documentation by being in the root of the repository e.g. ``` ~/modularis``` and typing the command:
```bash
./scripts/build_docs.sh
```
You can then view the documentation locally in your default browser with:
```bash
./scripts/view_docs.sh
```
### Notes
Run the command below in the docs folder of the repo if the built documentation is not updating.
```bash
make clean
```
This will remove all built files so the build will regenerate everything.